"""Utilities for web applications."""

import functools
import logging
import queue
import threading
from typing import Generic, TypeVar, Union

from flask import Response

FlaskResponse = Union[str, tuple[str, int], Response]


class QueueAbstractionClass:
    """Simplifies abstraction of calling functions through a queue interface."""

    def __init__(self) -> None:
        self._calls_queue = queue.Queue()
        self._logger = logging.getLogger("QueueAbstractionClass")

    @staticmethod
    def queue_abstract(fun):
        """Function wrapper to abstract a call through a queue.

        The wrapped function will only be handles by invoking the
        `_process_pending_calls` method.
        """

        @functools.wraps(fun)
        def inner(*args, **kwargs):
            return_queue = queue.Queue(maxsize=1)
            self: QueueAbstractionClass = args[0]
            self._logger.info(f"> Send call to {fun.__name__}")
            self._calls_queue.put((fun, args, kwargs, return_queue))
            return_value = return_queue.get()
            self._logger.info(f"< Received return from {fun.__name__}")
            if isinstance(return_value, Exception):
                raise return_value
            return return_value

        return inner

    def _process_pending_calls(self) -> None:
        """Process all pending function calls to functions wrapped with `queue_abstract`."""
        try:
            while True:
                method, args, kwargs, return_queue = self._calls_queue.get(block=False)
                try:
                    self._logger.info(f"> Received call to {method.__name__}")
                    return_queue.put(method(*args, **kwargs))
                except Exception as e:
                    return_queue.put(e)
                finally:
                    self._logger.info(f"< Sending return from {method.__name__}")
        except queue.Empty:
            pass


T = TypeVar("T")


class DataDispatcher(Generic[T]):
    """Thread save one-to-many distributor of objects."""

    def __init__(self) -> None:
        self._data = None
        self._data_lock = threading.Lock()
        self._data_changed = threading.Condition(lock=self._data_lock)

    def put(self, data: T) -> None:
        """Put an object to be consumed by all listeners."""
        self._data = data
        with self._data_lock:
            self._data_changed.notify_all()

    def get_blocking(self) -> T:
        """Get the next put object in blocking mode."""
        with self._data_lock:
            self._data_changed.wait()
        assert self._data is not None
        return self._data
