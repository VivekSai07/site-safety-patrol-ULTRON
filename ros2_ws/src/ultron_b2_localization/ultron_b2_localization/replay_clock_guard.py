"""Replay clock progress monitor using monotonic wall time."""


class ReplayClockGuard:
    def __init__(self, timeout_s, jump_s):
        if timeout_s <= 0 or jump_s <= 0:
            raise ValueError('clock watchdog timeout and jump limit must be positive')
        self.timeout_s = timeout_s
        self.jump_ns = int(jump_s * 1e9)
        self.last_ns = None
        self.last_progress_wall = None
        self.progressing = False

    def observe(self, stamp_ns, wall):
        previous = self.last_ns
        if stamp_ns <= 0:
            self.last_ns = None
            self.progressing = False
            return 'jump' if previous is not None else 'waiting'
        self.last_ns = stamp_ns
        if previous is None:
            self.last_progress_wall = wall
            self.progressing = False
            return 'waiting'
        delta = stamp_ns - previous
        if delta < 0 or delta > self.jump_ns:
            self.last_progress_wall = wall
            self.progressing = False
            return 'jump'
        if delta > 0:
            self.last_progress_wall = wall
            self.progressing = True
            return 'progress'
        return 'unchanged'

    def stalled(self, wall):
        return (not self.progressing or self.last_progress_wall is None
                or wall - self.last_progress_wall > self.timeout_s)
