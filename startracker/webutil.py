"""Utilities for web applications."""

import functools
import queue
import threading
from typing import Union

import cv2
from flask import Response

FlaskResponse = Union[str, tuple[str, int], Response]


class QueueAbstractionClass:
    """Simplifies abstraction of calling functions through a queue interface."""

    def __init__(self):
        self._calls_queue = queue.Queue()

    @staticmethod
    def queue_abstract(fun):
        @functools.wraps(fun)
        def inner(*args, **kwargs):
            return_queue = queue.Queue(maxsize=1)
            self: QueueAbstractionClass = args[0]
            self._calls_queue.put((fun, args, kwargs, return_queue))
            return_value = return_queue.get()
            if isinstance(return_value, Exception):
                raise return_value
            return return_value

        return inner

    def _process_pending_calls(self):
        """Process all pending function calls to functions wrapped with queue_abstract."""
        try:
            while True:
                method, args, kwargs, return_queue = self._calls_queue.get(block=False)
                try:
                    return_queue.put(method(*args, **kwargs))
                except Exception as e:
                    return_queue.put(e)
        except queue.Empty:
            pass


class ImageData:
    def __init__(self):
        self._image = None
        self._data_lock = threading.Lock()
        self._data_changed = threading.Condition(lock=self._data_lock)

    def put(self, image):
        success, encoded_array = cv2.imencode(".png", image)
        if not success:
            return
        image_data = encoded_array.tobytes()

        self._image = image_data
        with self._data_lock:
            self._data_changed.notify_all()

    def get_blocking(self):
        with self._data_lock:
            self._data_changed.wait()
        assert self._image is not None
        return self._image
