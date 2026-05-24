"""
Classifier module: training, persistence, and real-time prediction.

Primary algorithm: LinearDiscriminantAnalysis (LDA).
  - Sub-millisecond predict, natural posterior probabilities.
  - Works well with O(100–1000) training windows per class.

Fallback: SVM with RBF kernel (selected automatically in train() if LDA
cross-val accuracy < 85 %).
"""

from __future__ import annotations

from pathlib import Path
import warnings

import numpy as np
from numpy.typing import NDArray
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.exceptions import ConvergenceWarning
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

import joblib
from collections import deque

from .config import CONFIDENCE_THRESHOLD, GESTURE_NAMES, REST_LABEL, N_GESTURES

# Default filenames inside the model directory
SCALER_FILE = "scaler.pkl"
CLF_FILE = "classifier.pkl"
META_FILE = "meta.pkl"
_MODEL_PREFERENCE = ("MLP", "LDA", "SVM")


def _build_lda() -> Pipeline:
    return Pipeline([
        ("scaler", StandardScaler()),
        ("clf", LinearDiscriminantAnalysis(solver="svd", store_covariance=True)),
    ])


def _build_svm() -> Pipeline:
    return Pipeline([
        ("scaler", StandardScaler()),
        ("clf", SVC(kernel="rbf", probability=True, C=10.0, gamma="scale")),
    ])


def _build_mlp() -> Pipeline:
    return Pipeline([
        ("scaler", StandardScaler()),
        (
            "clf",
            MLPClassifier(
                hidden_layer_sizes=(128, 64),
                activation="relu",
                solver="adam",
                alpha=1e-4,
                batch_size="auto",
                learning_rate_init=1e-3,
                max_iter=400,
                early_stopping=True,
                n_iter_no_change=20,
                random_state=42,
            ),
        ),
    ])


def _pick_best_candidate(scores: dict[str, float]) -> tuple[str, float]:
    best_name = max(
        scores,
        key=lambda name: (scores[name], -_MODEL_PREFERENCE.index(name)),
    )
    return best_name, scores[best_name]


def _cross_validate_candidate(
    name: str,
    pipe: Pipeline,
    X: NDArray,
    y: NDArray,
    cv: StratifiedKFold,
    *,
    verbose: bool,
) -> tuple[float, NDArray]:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ConvergenceWarning)
        y_pred_cv = cross_val_predict(pipe, X, y, cv=cv)

    accuracy = float(np.mean(y_pred_cv == y))
    if verbose:
        print(f"\n{name} cross-val accuracy ({cv.n_splits}-fold): {accuracy * 100:.1f} %")
        _print_confusion(y, y_pred_cv)
    return accuracy, y_pred_cv


def train(
    X: NDArray,
    y: NDArray,
    *,
    model_dir: str | Path = "models",
    cv_folds: int = 5,
    verbose: bool = True,
    feature_set: str = "emg_only",
) -> Pipeline:
    """Train (and cross-validate) a classifier on feature matrix X and labels y.

    Args:
        X: (N_windows, N_features) feature matrix
        y: (N_windows,) integer labels
        model_dir: directory to save fitted pipeline
        cv_folds: number of stratified cross-validation folds
        verbose: print diagnostics
        feature_set: "emg_only" or "emg_imu" — recorded in meta.pkl

    Returns:
        Fitted scikit-learn Pipeline (scaler → classifier)
    """
    model_dir = Path(model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)

    cv = StratifiedKFold(n_splits=cv_folds, shuffle=True, random_state=42)
    candidate_builders = {
        "MLP": _build_mlp,
        "LDA": _build_lda,
        "SVM": _build_svm,
    }
    candidate_scores: dict[str, float] = {}
    candidate_pipes: dict[str, Pipeline] = {}

    for name in _MODEL_PREFERENCE:
        pipe = candidate_builders[name]()
        score, _ = _cross_validate_candidate(name, pipe, X, y, cv, verbose=verbose)
        candidate_scores[name] = score
        candidate_pipes[name] = pipe

    algo, best_score = _pick_best_candidate(candidate_scores)
    pipe = candidate_pipes[algo]

    # ── final fit on full dataset ─────────────────────────────────────────────
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ConvergenceWarning)
        pipe.fit(X, y)
    if verbose:
        print(f"\nFinal model: {algo} — fit on {len(y)} windows.")

    # ── save ─────────────────────────────────────────────────────────────────
    joblib.dump(pipe, model_dir / CLF_FILE)
    meta = {
        "algorithm": algo,
        "cv_accuracy": best_score,
        "candidate_scores": candidate_scores,
        "classes": np.unique(y).tolist(),
        "feature_set": feature_set,
        "feature_dim": int(X.shape[1]),
    }
    joblib.dump(meta, model_dir / META_FILE)

    if verbose:
        print(f"Saved classifier → {model_dir / CLF_FILE}")

    return pipe


def load(model_dir: str | Path = "models") -> Pipeline:
    """Load a previously saved classifier pipeline."""
    path = Path(model_dir) / CLF_FILE
    if not path.exists():
        raise FileNotFoundError(f"No classifier found at {path}. Run train.py first.")
    return joblib.load(path)


def predict(
    pipe: Pipeline,
    x: NDArray,
    *,
    threshold: float = CONFIDENCE_THRESHOLD,
) -> tuple[int, float, NDArray]:
    """Predict gesture label for a single feature vector.

    Args:
        pipe: fitted Pipeline from train() or load()
        x: 1-D feature vector (N_features,)
        threshold: confidence below which REST is returned

    Returns:
        (label, confidence, probabilities)
        - label: integer class label (REST if confidence < threshold)
        - confidence: probability of the predicted class
        - probabilities: full posterior vector of length N_GESTURES (0-indexed)
    """
    from .config import N_GESTURES

    x2d = x.reshape(1, -1)

    # Build a full N_GESTURES probability vector, mapping by actual class index
    raw_probs = pipe.predict_proba(x2d)[0]
    clf_classes = pipe.named_steps["clf"].classes_
    full_probs = np.zeros(N_GESTURES, dtype=float)
    for cls_idx, cls_label in enumerate(clf_classes):
        if 0 <= cls_label < N_GESTURES:
            full_probs[cls_label] = raw_probs[cls_idx]

    best = int(np.argmax(full_probs))
    conf = float(full_probs[best])

    if conf < threshold:
        return REST_LABEL, conf, full_probs

    return best, conf, full_probs


# ── Prediction smoother ───────────────────────────────────────────────────────

class PredictionSmoother:
    """Majority-vote smoother over a sliding window of recent predictions.

    Tie-breaking: on a tie the previous smoothed output is returned unchanged,
    avoiding the implicit bias toward label 0 that np.argmax produces.
    """

    def __init__(self, window: int = 5) -> None:
        self._window = window
        self._history: deque[int] = deque(maxlen=window)
        self._last_output: int = 0

    def update(self, label: int) -> int:
        """Add a new raw prediction and return the smoothed label."""
        self._history.append(label)
        counts = np.bincount(list(self._history), minlength=len(GESTURE_NAMES))
        majority = int(np.argmax(counts))
        # On a tie, argmax returns the lowest-index label — keep previous instead
        if counts[majority] * 2 > len(self._history):   # strict majority
            self._last_output = majority
        # else: leave _last_output unchanged (tie → hold)
        return self._last_output


# ── helpers ───────────────────────────────────────────────────────────────────

def _print_confusion(y_true: NDArray, y_pred: NDArray) -> None:
    labels = list(range(len(GESTURE_NAMES)))
    cm = confusion_matrix(y_true, y_pred, labels=labels)
    print("\nConfusion matrix (rows=true, cols=pred):")
    header = "       " + "  ".join(f"{GESTURE_NAMES[i]:>6}" for i in labels)
    print(header)
    for i, row in enumerate(cm):
        cells = "  ".join(f"{v:6d}" for v in row)
        print(f"  {GESTURE_NAMES[i]:>5}  {cells}")
    print()
    print(classification_report(y_true, y_pred,
                                 labels=labels,
                                 target_names=GESTURE_NAMES,
                                 zero_division=0))
