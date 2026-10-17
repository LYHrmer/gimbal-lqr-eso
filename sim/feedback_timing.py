"""Deterministic acquisition/delivery clocks, independent of controller equations.

Times are integer ticks on the simulator's control grid. Position and velocity
can have separate producers. A read does not acquire another noisy measurement.
"""
from dataclasses import asdict, dataclass
import heapq

import numpy as np


@dataclass(frozen=True)
class ChannelTiming:
    period_samples: int = 1
    delay_samples: int = 0
    phase_samples: int = 0
    delay_jitter_samples: tuple = ()
    drop_every: int = 0

    def __post_init__(self):
        values = (self.period_samples, self.delay_samples, self.phase_samples, self.drop_every)
        if (any(type(v) is not int for v in values) or self.period_samples < 1 or
                self.delay_samples < 0 or not 0 <= self.phase_samples < self.period_samples or
                self.drop_every < 0 or
                any(type(v) is not int or self.delay_samples+v < 0
                    for v in self.delay_jitter_samples)):
            raise ValueError("invalid integer feedback clock")


class FeedbackSampler:
    """Hold source timestamps and reject older packets arriving out of order.

    Call once per consecutive control tick, starting at zero. Both channels
    start with one synchronized snapshot at tick zero. Afterwards
    all delays/drops apply. Noise uses a pair of draws on each acquisition tick
    (including dropped acquisitions), matching the previous synchronous runner.
    """
    def __init__(self, position, velocity, seed, noise_std, quantum):
        self.clocks = (position, velocity)
        self.rng = np.random.default_rng(seed)
        self.noise_std, self.quantum = noise_std, quantum
        self.pending = [[], []]
        self.held = [None, None]
        self.sequence = [0, 0]
        self.dropped = [0, 0]
        self.discarded_old = [0, 0]
        self.max_age = [0, 0]
        self.last_tick = -1

    def read(self, tick, position, velocity):
        if type(tick) is not int or tick != self.last_tick+1:
            raise ValueError("feedback clock requires every consecutive tick, starting at zero")
        self.last_tick = tick
        due = [tick == 0 or (tick >= c.phase_samples and
                (tick-c.phase_samples) % c.period_samples == 0) for c in self.clocks]
        noise = self.rng.normal(size=2) if any(due) else (0., 0.)
        for axis, (clock, value) in enumerate(zip(self.clocks, (position, velocity))):
            if due[axis]:
                value += noise[axis]*self.noise_std[axis]
                if self.quantum[axis]:
                    value = float(np.round(value/self.quantum[axis])*self.quantum[axis])
                if tick == 0:
                    self.held[axis] = (tick, value)
                else:
                    self.sequence[axis] += 1
                    seq = self.sequence[axis]
                    if clock.drop_every and seq % clock.drop_every == 0:
                        self.dropped[axis] += 1
                    else:
                        jitter = (clock.delay_jitter_samples[(seq-1) % len(clock.delay_jitter_samples)]
                                  if clock.delay_jitter_samples else 0)
                        heapq.heappush(self.pending[axis], (tick+clock.delay_samples+jitter, tick, value))
            while self.pending[axis] and self.pending[axis][0][0] <= tick:
                _, source, value = heapq.heappop(self.pending[axis])
                if source > self.held[axis][0]:
                    self.held[axis] = (source, value)
                else:
                    self.discarded_old[axis] += 1
            self.max_age[axis] = max(self.max_age[axis], tick-self.held[axis][0])
        return tuple(v for _, v in self.held), tuple(t for t, _ in self.held)

    def metrics(self, dt):
        return {"position": asdict(self.clocks[0]), "velocity": asdict(self.clocks[1]),
                "startup": "synchronized t=0 snapshot; later acquisitions follow independent clocks",
                "max_position_age_s": self.max_age[0]*dt,
                "max_velocity_age_s": self.max_age[1]*dt,
                "dropped_acquisitions": self.dropped.copy(),
                "discarded_out_of_order": self.discarded_old.copy(),
                "api_age_policy": "max(position age, velocity age); preserves the oldest input age"}
