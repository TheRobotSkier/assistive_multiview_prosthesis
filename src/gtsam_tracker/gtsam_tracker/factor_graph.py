"""factor_graph — GTSAM factor graph construction for trajectory estimation.

ZERO ROS imports.  Uses ``gtsam`` and ``numpy`` only.

The factor graph maintains two independent pose chains:

- **head** chain (keys ``gtsam.symbol('h', i)``): head IMU pose in ``marker_map``.
- **arm** chain (keys ``gtsam.symbol('a', i)``): arm IMU pose in ``marker_map``.

Factor types:
- Odometry between-factors (consecutive poses on each chain).
- ArUco prior factors (absolute pose constraint on a single key).
- Visual between-factors (cross-chain relative pose from SIFT/SuperPoint).
- Range factors (soft kinematic distance constraint between head and arm).

The smoother uses incremental ISAM2 with configurable relinearisation
parameters.  A fixed-lag window is maintained by tracking key timestamps and
marginalising keys older than the lag.

Reference: V6 plan §5.3, §5.4.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

import gtsam

__all__ = [
    "TrajectoryFactorGraph",
    "noise_from_covariance_diag",
    "default_odom_noise",
]


# ---------------------------------------------------------------------------
# Noise model helpers
# ---------------------------------------------------------------------------

def noise_from_covariance_diag(cov_diag_6) -> gtsam.noiseModel:
    """Build a diagonal GTSAM noise model from a 6-element covariance diagonal.

    Parameters
    ----------
    cov_diag_6 : array-like of 6 floats
        Variances ``[sx2, sy2, sz2, sr2, sp2, sy2]`` (x, y, z, roll, pitch, yaw).
    """
    sigmas = np.sqrt(np.asarray(cov_diag_6, dtype=np.float64))
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


def default_odom_noise(sigma_t: float = 0.01, sigma_r: float = 0.01) -> gtsam.noiseModel:
    """Fixed diagonal noise model when odom covariance is unavailable.

    Parameters
    ----------
    sigma_t : float
        Translation sigma in metres (applied to x, y, z).
    sigma_r : float
        Rotation sigma in radians (applied to roll, pitch, yaw).
    """
    sigmas = np.array([sigma_t] * 3 + [sigma_r] * 3)
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


# ---------------------------------------------------------------------------
# TrajectoryFactorGraph
# ---------------------------------------------------------------------------

class TrajectoryFactorGraph:
    """Two-chain (head + arm) GTSAM incremental smoother.

    Uses ISAM2 for incremental optimisation with a fixed-lag window.
    Keys older than ``lag_s`` seconds are marginalised on each update.

    Parameters
    ----------
    lag_s : float
        Smoother lag in seconds.  Keys older than this are marginalised.
    """

    def __init__(self, lag_s: float = 7.0):
        self._lag_s = lag_s

        # ISAM2 parameters
        isam_params = gtsam.ISAM2Params()
        # relinearizeThreshold = 0.01 — at 12 Hz with <1 cm/step VIO deltas,
        # variables accumulate ~5 cm of linearization error between checks
        # (every 5 updates = ~415 ms) and are relinearized promptly.
        # This keeps the linearized-system approximation accurate enough
        # that Gauss-Newton converges in few iterations.
        # Setting this too loose (e.g. GTSAM default 0.1) makes the
        # linearized system stale → more solver iterations → LOWER output
        # rate, as confirmed by the iteration-13 regression.
        isam_params.setRelinearizeThreshold(0.01)
        isam_params.relinearizeSkip = 5
        self._isam = gtsam.ISAM2(isam_params)

        # Track the last key index and timestamp for each chain.
        self._last_key: Dict[str, Optional[int]] = {"h": None, "a": None}
        self._last_stamp: Dict[str, Optional[float]] = {"h": None, "a": None}

        # Accumulate new factors + initial estimates between updates.
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()

        # Track which symbol keys have initial estimates.
        self._initialized: set = set()

        # Track all key timestamps for marginalisation.
        self._key_timestamps: Dict[int, float] = {}

        # Track the set of all keys ever inserted.
        self._all_keys: set = set()

    # ------------------------------------------------------------------
    # Key generation
    # ------------------------------------------------------------------

    @staticmethod
    def _make_key(chain: str, index: int) -> int:
        """Generate a GTSAM symbol key.

        ``chain`` is ``'h'`` for head or ``'a'`` for arm.
        """
        return gtsam.symbol(chain, index)

    # ------------------------------------------------------------------
    # Odometry between-factors
    # ------------------------------------------------------------------

    def add_odometry_factor(
        self,
        key_head: int,
        key_arm: int,
        stamp: float,
        delta_head: np.ndarray,
        delta_arm: np.ndarray,
        noise_head: gtsam.noiseModel,
        noise_arm: gtsam.noiseModel,
        init_head: Optional[gtsam.Pose3] = None,
        init_arm: Optional[gtsam.Pose3] = None,
    ) -> None:
        """Add odometry between-factors for both chains.

        Parameters
        ----------
        key_head, key_arm : int
            The *current* (new) key indices for the head and arm chains.
        stamp : float
            Timestamp in seconds for the new keys.
        delta_head, delta_arm : ndarray (4, 4)
            The relative pose delta ``T_prev_curr`` for each chain.
        noise_head, noise_arm : gtsam.noiseModel
            Noise models for the head and arm between-factors.
        init_head, init_arm : gtsam.Pose3, optional
            Pre-computed initial estimates for the new keys.  When provided,
            these are used directly instead of computing ``get_pose(prev_key)
            * delta``, which avoids a redundant ISAM2 back-substitution.
            Typically set to the raw odometry pose for the current step.
            Falls back to the existing get_pose-based initialisation when
            ``None`` (backward compatible).
        """
        for chain, key_new, delta, noise in [
            ("h", key_head, delta_head, noise_head),
            ("a", key_arm, delta_arm, noise_arm),
        ]:
            delta_pose = gtsam.Pose3(np.asarray(delta, dtype=np.float64))

            key_last = self._last_key[chain]
            k_new = self._make_key(chain, key_new)

            if key_last is not None:
                # BetweenFactor(last, new, delta)
                k_last = self._make_key(chain, key_last)
                self._new_factors.add(
                    gtsam.BetweenFactorPose3(k_last, k_new, delta_pose, noise)
                )
            else:
                # First key on this chain: add a prior.
                prior_noise = noise_from_covariance_diag([1e-6] * 6)
                self._new_factors.add(
                    gtsam.PriorFactorPose3(k_new, delta_pose, prior_noise)
                )

            # Provide initial estimate if not yet in the graph.
            if k_new not in self._initialized:
                # When a pre-computed initial estimate is provided (e.g. the
                # raw odometry pose from the caller), use it directly to avoid
                # a redundant get_pose() / calculateEstimatePose3() call.
                if (chain == "h" and init_head is not None):
                    init_pose = init_head
                elif (chain == "a" and init_arm is not None):
                    init_pose = init_arm
                elif key_last is not None and self._make_key(chain, key_last) in self._initialized:
                    prev_pose = self.get_pose(self._make_key(chain, key_last))
                    init_pose = prev_pose.compose(delta_pose)
                else:
                    init_pose = delta_pose
                self._new_values.insert(k_new, init_pose)
                self._initialized.add(k_new)
                self._all_keys.add(k_new)

            self._last_key[chain] = key_new
            self._last_stamp[chain] = stamp
            self._key_timestamps[k_new] = stamp

    # ------------------------------------------------------------------
    # ArUco prior factor
    # ------------------------------------------------------------------

    def add_aruco_prior(
        self,
        key: int,
        T_map_imu: np.ndarray,
        covariance_diag,
    ) -> None:
        """Add an absolute prior factor (from ArUco observation).

        Parameters
        ----------
        key : int
            The GTSAM symbol key for this prior.
        T_map_imu : ndarray (4, 4)
            The measured absolute pose in ``marker_map``.
        covariance_diag : array-like of 6 floats
            Diagonal covariance ``[sx2, sy2, sz2, sr2, sp2, sy2]``.
        """
        noise = noise_from_covariance_diag(covariance_diag)
        measured = gtsam.Pose3(np.asarray(T_map_imu, dtype=np.float64))
        self._new_factors.add(gtsam.PriorFactorPose3(key, measured, noise))

    # ------------------------------------------------------------------
    # Visual between-factor (cross-chain)
    # ------------------------------------------------------------------

    def add_visual_between(
        self,
        key_head: int,
        key_arm: int,
        T_head_arm: np.ndarray,
        covariance,
    ) -> None:
        """Add a cross-chain between-factor from visual observation.

        Parameters
        ----------
        key_head, key_arm : int
            Head and arm keys at the same timestamp.
        T_head_arm : ndarray (4, 4)
            Relative transform ``T_head_arm`` (from head to arm).
        covariance : array-like of 6 floats or gtsam.noiseModel
            Either a 6-element diagonal covariance or a noise model.
        """
        if isinstance(covariance, gtsam.noiseModel.Base):
            noise = covariance
        else:
            noise = noise_from_covariance_diag(covariance)

        measured = gtsam.Pose3(np.asarray(T_head_arm, dtype=np.float64))
        self._new_factors.add(
            gtsam.BetweenFactorPose3(key_head, key_arm, measured, noise)
        )

    # ------------------------------------------------------------------
    # Range factor (kinematic distance constraint)
    # ------------------------------------------------------------------

    def add_range_factor(
        self,
        key_head: int,
        key_arm: int,
        max_distance: float,
        sigma: float,
        T_head_arm: Optional[np.ndarray] = None,
    ) -> None:
        """Add a soft range constraint between head and arm.

        Adds a weak between-factor that constrains the head-arm relative
        pose to stay near the currently-observed configuration, preventing
        the two chains from drifting apart while respecting that the cameras
        move relative to each other (the arm is not rigidly attached to the
        head).

        Parameters
        ----------
        key_head, key_arm : int
            Head and arm keys.
        max_distance : float
            Approximate maximum head-arm separation in metres.  Used as
            the translation noise sigma so the constraint only activates
            when the chains diverge beyond this distance.
        sigma : float
            Rotation noise sigma in radians.
        T_head_arm : ndarray (4, 4) or None
            Current odometry-based relative transform T_head_arm.
            If None, falls back to a weak identity prior.
        """
        # Translation noise = max_distance (loose — only activates when
        # chains diverge by more than ~max_distance).  Rotation noise = sigma
        # (default 0.05 rad) so the factor provides a moderate relative
        # orientation constraint between the two kinematic chains.
        #
        # GTSAM BetweenFactorPose3 error ordering: [tx, ty, tz, rx, ry, rz].
        # First three = translation (sigma = max_distance), last three =
        # rotation (sigma = sigma parameter).
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([float(max_distance), float(max_distance), float(max_distance),
                      float(sigma), float(sigma), float(sigma)])
        )

        if T_head_arm is not None:
            measured = gtsam.Pose3(np.asarray(T_head_arm, dtype=np.float64))
        else:
            # No odometry reference — fall back to a very loose identity
            # prior.  The large sigmas make this a rubber-band that only
            # activates at multi-metre divergence.
            measured = gtsam.Pose3()

        self._new_factors.add(
            gtsam.BetweenFactorPose3(key_head, key_arm, measured, noise)
        )

    # ------------------------------------------------------------------
    # Reset (recovery from corrupted ISAM2 state)
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reinitialize the ISAM2 smoother, discarding all factors and values.

        Call this after an ``IndeterminantLinearSystemException`` (or any other
        corruption) leaves the graph in an unrecoverable state.  After reset,
        the next :meth:`add_odometry_factor` call will seed a fresh graph with
        a prior on the first pose.

        Reference: V6 §9 graceful-degradation / fault-tolerance.
        """
        isam_params = gtsam.ISAM2Params()
        # relinearizeThreshold = 0.01 — matches __init__.  Skip 5.
        isam_params.setRelinearizeThreshold(0.01)
        isam_params.relinearizeSkip = 5
        self._isam = gtsam.ISAM2(isam_params)

        self._last_key = {"h": None, "a": None}
        self._last_stamp = {"h": None, "a": None}

        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()

        self._initialized = set()
        self._key_timestamps = {}
        self._all_keys = set()

    # ------------------------------------------------------------------
    # Update / optimise
    # ------------------------------------------------------------------

    def update(self, marginalize_keys: Optional[List[int]] = None) -> None:
        """Run an ISAM2 update with all accumulated new factors.

        Parameters
        ----------
        marginalize_keys : list of int, optional
            Keys to marginalize during this update (fixed-lag smoother).
            Passed as ``marginalizeTheta`` to ISAM2 so old variables are
            removed and replaced by linear approximation factors.
        """
        if self._new_factors.size() == 0 and not marginalize_keys:
            return

        # Build marginalization KeyVector if keys were provided.
        if marginalize_keys:
            theta = gtsam.KeyVector()
            for k in marginalize_keys:
                theta.push_back(k)
        else:
            theta = gtsam.KeyVector()

        self._isam.update(self._new_factors, self._new_values, theta)

        # Reset accumulators.
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()

    # ------------------------------------------------------------------
    # Marginalisation (fixed-lag)
    # ------------------------------------------------------------------

    def marginalize_old_keys(self, current_stamp: float) -> List[int]:
        """Identify keys older than ``current_stamp - lag_s`` for marginalisation.

        Returns the list of integer keys eligible for marginalisation.  The
        caller should pass these keys to :meth:`update` so ISAM2 removes
        them from the Bayes tree and replaces them with linear approximation
        factors (fixed-lag smoother behaviour).

        Parameters
        ----------
        current_stamp : float
            Current timestamp in seconds.  Keys with timestamps older than
            ``current_stamp - lag_s`` are returned.

        Returns
        -------
        list of int
            Keys that should be marginalised on the next update.
        """
        threshold = current_stamp - self._lag_s
        to_remove = [
            k for k, t in self._key_timestamps.items()
            if t < threshold
        ]
        if not to_remove:
            return []

        # Remove from timestamp tracking (keys are gone from the smoother
        # once ISAM2 marginalises them during update()).
        for k in to_remove:
            self._key_timestamps.pop(k, None)
            self._all_keys.discard(k)
            # Also forget initialisation status so re-adding a key with
            # the same index after a reset works cleanly.
            self._initialized.discard(k)
        return to_remove

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def get_pose(self, key: int) -> gtsam.Pose3:
        """Query the current estimate for a key.

        Parameters
        ----------
        key : int
            GTSAM symbol key.

        Returns
        -------
        gtsam.Pose3
            The current best estimate.
        """
        return self._isam.calculateEstimatePose3(key)

    def get_all_estimates(self) -> gtsam.Values:
        """Return the full current estimate."""
        return self._isam.calculateEstimate()

    @property
    def lag_s(self) -> float:
        return self._lag_s
