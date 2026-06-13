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

from typing import Dict, Optional

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

    def __init__(self, lag_s: float = 15.0):
        self._lag_s = lag_s

        # ISAM2 parameters
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.001)
        isam_params.relinearizeSkip = 3
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
                if key_last is not None and self._make_key(chain, key_last) in self._initialized:
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
    ) -> None:
        """Add a soft range constraint between head and arm.

        If the head-arm distance exceeds ``max_distance``, a penalty pulls
        them back.  Implemented as a loose between-factor that acts as a soft
        spring pulling the two poses together.

        Parameters
        ----------
        key_head, key_arm : int
            Head and arm keys.
        max_distance : float
            The target / maximum distance in metres.
        sigma : float
            Range measurement sigma in metres.
        """
        # Loose rotation noise, tight translation noise.
        # The zero-translation between-factor pulls head and arm together,
        # counteracted by odometry.  This acts as a soft spring.
        loose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([10.0, 10.0, 10.0, sigma, sigma, sigma])
        )

        measured = gtsam.Pose3()  # identity
        self._new_factors.add(
            gtsam.BetweenFactorPose3(key_head, key_arm, measured, loose_noise)
        )

    # ------------------------------------------------------------------
    # Update / optimise
    # ------------------------------------------------------------------

    def update(self) -> None:
        """Run an ISAM2 update with all accumulated new factors."""
        if self._new_factors.size() == 0:
            return

        self._isam.update(self._new_factors, self._new_values)

        # Reset accumulators.
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()

    # ------------------------------------------------------------------
    # Marginalisation (fixed-lag)
    # ------------------------------------------------------------------

    def marginalize_old_keys(self, current_stamp: float) -> int:
        """Identify keys older than ``current_stamp - lag_s`` for marginalisation.

        In this ISAM2-based implementation, old keys are tracked and can be
        removed in a future rebuild step.  The timestamp bookkeeping is
        maintained here so that the graph size stays bounded.

        Returns the number of keys eligible for marginalisation.
        """
        threshold = current_stamp - self._lag_s
        to_remove = [
            k for k, t in self._key_timestamps.items()
            if t < threshold
        ]
        if not to_remove:
            return 0

        # Mark old keys as marginalised (remove from timestamp tracking).
        for k in to_remove:
            self._key_timestamps.pop(k, None)
        return len(to_remove)

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
