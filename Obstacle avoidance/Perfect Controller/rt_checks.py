# rt_checks.py
from typing import Iterable
from math import inf
from rt_types import Sample

def assert_no_collision(samples: Iterable[Sample]) -> None:
    """No sample should report a collision."""
    assert all(not s.collision for s in samples), "Collision detected"

def assert_min_clearance(samples: Iterable[Sample], threshold_px: float) -> None:
    """Closest approach must be >= threshold."""
    min_d = min((s.closest_obs for s in samples), default=inf)
    assert min_d >= threshold_px, f"Min clearance {min_d:.1f}px below {threshold_px}px"

# def assert_avoids_within_T(samples: Iterable[Sample], T: float) -> None:
#     """First avoid_active must start within T seconds from first sample."""
#     items = list(samples)
#     if not items:
#         return
#     t0 = items[0].t
#     t_first = next((s.t for s in items if s.avoid_active), None)
#     assert t_first is not None and (t_first - t0) <= T, f"No avoidance within {T}s"

def assert_avoids_within_T(samples: Iterable[Sample], T: float) -> None:
    items = list(samples)          # materialize the iterable (we may need to scan twice)
    if not items:                  # no data -> nothing to assert (silently succeeds)
        return
    t0 = items[0].t                # timestamp of the FIRST sample in the list
    t_first = next(                # find the FIRST sample where avoid_active == True
        (s.t for s in items if s.avoid_active),
        None
    )
    assert t_first is not None and (t_first - t0) <= T, \
        f"No avoidance within {T}s"

# def assert_recovers_within_N(samples: Iterable[Sample], N: float) -> None:
#     """From avoid start to first non-avoid must be <= N seconds."""
#     items = list(samples)
#     t_start = next((s.t for s in items if s.avoid_active), None)
#     if t_start is None:
#         return  # no avoidance -> nothing to check
#     t_end = next((s.t for s in items if s.t >= t_start and not s.avoid_active), None)
#     assert t_end is not None and (t_end - t_start) <= N, f"Recovery exceeded {N}s"

def assert_recovers_within_N(samples: Iterable[Sample], N: float) -> None:
    items = sorted(samples, key=lambda s: s.t)
    if not items:
        return

    # First time avoidance becomes active in this window
    t_start = next((s.t for s in items if s.avoid_active), None)
    if t_start is None:
        return  # no avoidance at all in this window -> nothing to check

    # First time it turns off after it started
    t_end = next((s.t for s in items if s.t >= t_start and not s.avoid_active), None)

    # If it hasn't turned off yet, decide if we can judge now
    t_last = items[-1].t
    if t_end is None:
        # If the window hasn't extended beyond N seconds past t_start, it's inconclusive -> don't fail
        if (t_last - t_start) < N:
            return
        # Otherwise we've observed >N seconds of continuous avoidance without recovery -> fail
        raise AssertionError(f"Recovery exceeded {N}s")

    # It did turn off; check duration
    if (t_end - t_start) > N:
        raise AssertionError(f"Recovery exceeded {N}s")