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

import numpy as np
from numpy.typing import NDArray
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

import joblib

from .config import CONFIDENCE_THRESHOLD, GESTURE_NAMES, REST_LABEL

# Default filenames inside the model directory
SCALER_FILE = "scaler.pkl"
CLF_FILE = "classifier.pkl"
META_FILE = "meta.pkl"


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


def train(
    X: NDArray,
    y: NDArray,
    *,
    model_dir: str | Path = "models",
    cv_folds: int = 5,
    verbose: bool = True,
) -> Pipeline:
    """Train (and cross-validate) a classifier on feature matrix X and labels y.

    Args:
        X: (N_windows, N_features) feature matrix
        y: (N_windows,) integer labels
        model_dir: directory to save fitted pipeline
        cv_folds: number of stratified cross-validation folds
        verbose: print diagnostics

    Returns:
        Fitted scikit-learn Pipeline (scaler → classifier)
    """
    model_dir = Path(model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)

    # ── cross-validate LDA ───────────────────────────────────────────────────
    pipe_lda = _build_lda()
    cv = StratifiedKFold(n_splits=cv_folds, shuffle=True, random_state=42)
    y_pred_cv = cross_val_predict(pipe_lda, X, y, cv=cv)
    lda_acc = float(np.mean(y_pred_cv == y))

    if verbose:
        print(f"\nLDA cross-val accuracy ({cv_folds}-fold): {lda_acc * 100:.1f} %")
        _print_confusion(y, y_pred_cv)

    # ── choose model ─────────────────────────────────────────────────────────
    if lda_acc >= 0.85:
        pipe = pipe_lda
        algo = "LDA"
    else:
        if verbose:
            print(f"LDA accuracy < 85 %, switching to SVM …")
        pipe_svm = _build_svm()
        y_pred_svm = cross_val_predict(pipe_svm, X, y, cv=cv)
        svm_acc = float(np.mean(y_pred_svm == y))
        if verbose:
            print(f"SVM cross-val accuracy ({cv_folds}-fold): {svm_acc * 100:.1f} %")
            _print_confusion(y, y_pred_svm)
        pipe = pipe_svm if svm_acc >= lda_acc else pipe_lda
        algo = "SVM" if svm_acc >= lda_acc else "LDA"

    # ── final fit on full dataset ─────────────────────────────────────────────
    pipe.fit(X, y)
    if verbose:
        print(f"\nFinal model: {algo} — fit on {len(y)} windows.")

    # ── save ─────────────────────────────────────────────────────────────────
    joblib.dump(pipe, model_dir / CLF_FILE)
    meta = {"algorithm": algo, "cv_accuracy": lda_acc, "classes": np.unique(y).tolist()}
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
    """Majority-vote smoother over a sliding window of recent predictions."""

    def __init__(self, window: int = 3) -> None:
        self._window = window
        self._history: list[int] = []

    def update(self, label: int) -> int:
        """Add a new raw prediction and return the smoothed label."""
        self._history.append(label)
        if len(self._history) > self._window:
            self._history.pop(0)
        counts = np.bincount(self._history, minlength=len(GESTURE_NAMES))
        return int(np.argmax(counts))


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
