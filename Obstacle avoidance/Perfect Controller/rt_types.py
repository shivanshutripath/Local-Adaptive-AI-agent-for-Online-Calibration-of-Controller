# rt_types.py
from dataclasses import dataclass
from collections import deque
from typing import Deque, Optional, Iterable

@dataclass
class Sample:
    t: float          # time (s)
    x: float          # px
    y: float          # px
    theta: float      # rad
    vl: float         # px/s (applied)
    vr: float         # px/s (applied)
    closest_obs: float  # px (inf if none)
    avoid_active: bool   # your controller flag
    collision: bool      # your collision flag

class Ring:
    """Fixed-size rolling window of Samples."""
    def __init__(self, capacity: int = 600):
        self._q: Deque[Sample] = deque(maxlen=capacity)

    def push(self, s: Sample) -> None:
        self._q.append(s)

    def last(self, n: int) -> Iterable[Sample]:
        # Return the last n samples (or all if fewer exist)
        q = list(self._q)
        return q[-n:] if n <= len(q) else q

    def all(self) -> Iterable[Sample]:
        return list(self._q)

    def latest(self) -> Optional[Sample]:
        return self._q[-1] if self._q else None
