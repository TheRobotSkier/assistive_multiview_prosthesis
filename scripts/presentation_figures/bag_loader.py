"""MCAP bag loading for odom trajectory data, with on-disk caching.

Reads ``nav_msgs/msg/Odometry`` pose data from all v6 bags in
``data/bags/``.  Returns per-bag x/y arrays for the head and arm odom
topics::

    /jetson/head/odom
    /jetson/arm/odom

Decoding 20+ MCAP bags is slow (minutes), so a compact ``.npz`` cache is
written after the first successful decode.  Subsequent calls load the
cache in milliseconds unless the bag set has changed (detected via a
manifest of file sizes / modification times).
"""
import glob
import json
import os

import numpy as np

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BAGS_DIR = os.path.join(SCRIPT_DIR, "..", "..", "data", "bags")
CACHE_DIR = os.path.join(SCRIPT_DIR, "output")
CACHE_PATH = os.path.join(CACHE_DIR, "odom_cache.npz")

HEAD_TOPIC = "/jetson/head/odom"
ARM_TOPIC = "/jetson/arm/odom"


def _iter_v6_bag_dirs():
    """Yield absolute paths to every ``v6_*`` bag directory."""
    for path in sorted(glob.glob(os.path.join(BAGS_DIR, "v6_*"))):
        if os.path.isdir(path):
            yield path


# ----------------------------------------------------------------------- #
#  Cache manifest                                                          #
# ----------------------------------------------------------------------- #
def _build_manifest():
    """Build a manifest of all v6 bags keyed by (path, size, mtime).

    Used to detect whether the cache is stale (bags added / removed /
    modified).  Values are JSON-native lists ``[size, mtime]`` so the
    manifest survives a JSON round-trip through the ``.npz`` cache
    unchanged.
    """
    entries = {}
    for bag_dir in _iter_v6_bag_dirs():
        for mcap_path in sorted(glob.glob(os.path.join(bag_dir, "*.mcap"))):
            st = os.stat(mcap_path)
            entries[mcap_path] = [st.st_size, int(st.st_mtime)]
    return entries


def _cache_is_valid(manifest):
    """Return True if the cache exists and matches *manifest*."""
    if not os.path.isfile(CACHE_PATH):
        return False
    try:
        with np.load(CACHE_PATH, allow_pickle=True) as data:
            stored_manifest = json.loads(str(data["_manifest"]))
        return stored_manifest == manifest
    except Exception:
        return False


# ----------------------------------------------------------------------- #
#  Save / load cache                                                       #
# ----------------------------------------------------------------------- #
def _save_cache(results, manifest):
    """Serialize *results* dict to the ``.npz`` cache."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    arrays = {"_manifest": np.array(json.dumps(manifest))}
    for bag_name, cams in results.items():
        for cam, axes in cams.items():
            for axis in ("x", "y", "z"):
                key = f"{bag_name}__{cam}__{axis}"
                arrays[key] = np.asarray(axes.get(axis, []), dtype=float)
    np.savez(CACHE_PATH, **arrays)


def _load_cache():
    """Rebuild the results dict from the ``.npz`` cache."""
    results = {}
    with np.load(CACHE_PATH, allow_pickle=True) as data:
        keys = [k for k in data.files if k != "_manifest"]
        for key in keys:
            parts = key.split("__")
            if len(parts) != 3:
                continue
            bag_name, cam, axis = parts
            bag = results.setdefault(bag_name, {})
            cam_d = bag.setdefault(cam, {"x": [], "y": [], "z": []})
            cam_d[axis] = data[key].tolist()
    return results


# ----------------------------------------------------------------------- #
#  Raw MCAP decode                                                         #
# ----------------------------------------------------------------------- #
def load_odom_from_bags(verbose=True):
    """Read odom trajectories from all v6 bags (slow — no cache).

    Returns a dict keyed by bag directory name, each value a dict::

        {
            "head": {"x": [...], "y": [...], "z": [...]},
            "arm":  {"x": [...], "y": [...], "z": [...]},
        }

    Bags that lack the jetson odom topics or fail to decode are skipped
    with a warning.
    """
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        raise RuntimeError(
            "mcap / mcap-ros2-support not installed — cannot read bags"
        )

    results = {}
    n_ok = 0
    n_skip = 0

    for bag_dir in _iter_v6_bag_dirs():
        bag_name = os.path.basename(bag_dir)
        mcap_files = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
        if not mcap_files:
            n_skip += 1
            continue

        head_xyz = {"x": [], "y": [], "z": []}
        arm_xyz = {"x": [], "y": [], "z": []}

        try:
            for mcap_path in mcap_files:
                with open(mcap_path, "rb") as f:
                    reader = make_reader(
                        f, decoder_factories=[DecoderFactory()]
                    )
                    for schema, channel, message, ros_msg in (
                        reader.iter_decoded_messages()
                    ):
                        topic = channel.topic
                        if topic == HEAD_TOPIC:
                            tgt = head_xyz
                        elif topic == ARM_TOPIC:
                            tgt = arm_xyz
                        else:
                            continue
                        try:
                            pos = ros_msg.pose.pose.position
                            tgt["x"].append(float(pos.x))
                            tgt["y"].append(float(pos.y))
                            tgt["z"].append(float(pos.z))
                        except (AttributeError, TypeError):
                            pass
        except Exception as e:
            if verbose:
                print(f"  [bag_io] {bag_name}: decode failed ({e})")
            n_skip += 1
            continue

        # Only keep bags that actually have data for at least one topic
        if head_xyz["x"] or arm_xyz["x"]:
            results[bag_name] = {"head": head_xyz, "arm": arm_xyz}
            n_ok += 1
        else:
            n_skip += 1

    if verbose:
        print(
            f"  [bag_io] decoded {n_ok} bags ({n_skip} skipped) "
            f"from {BAGS_DIR}"
        )
    return results


# ----------------------------------------------------------------------- #
#  Public API: cached loader                                               #
# ----------------------------------------------------------------------- #
def load_odom_cached(force_rebuild=False, verbose=True):
    """Load odom trajectories, using the on-disk cache when possible.

    Parameters
    ----------
    force_rebuild : bool
        If True, ignore any existing cache and decode all bags fresh.
    verbose : bool
        Print progress messages.
    """
    manifest = _build_manifest()

    if not force_rebuild and _cache_is_valid(manifest):
        if verbose:
            print(f"  [bag_io] loading odom cache ({CACHE_PATH})")
        return _load_cache()

    if verbose:
        print("  [bag_io] cache missing/stale — decoding MCAP bags ...")
    results = load_odom_from_bags(verbose=verbose)
    _save_cache(results, manifest)
    if verbose:
        print(f"  [bag_io] cache saved to {CACHE_PATH}")
    return results
