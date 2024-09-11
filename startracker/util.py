"""General utility functions."""

import contextlib
import time


@contextlib.contextmanager
def max_rate(max_rate_hz: float):
    """Pad a block of code with sleep, such that maximum rate is not exceeded."""
    t = time.monotonic() + 1 / max_rate_hz
    yield
    if (diff := t - time.monotonic()) > 0:
        time.sleep(diff)
