"""Utilities for web applications"""

import abc
import queue
import threading
import functools

import cv2


class QueueAbstractClass(abc.ABC):
    """
    Simplifies abstraction of calling functions through a queue interface.
    """

    def __init__(self):
        self._calls_queue = queue.Queue()

    @functools.wraps
    def queue_abstract(fun):
        def inner(*args):
            return_queue = queue.Queue(maxsize=1)
            self: QueueAbstractClass = args[0]
            self._calls_queue.put((fun, args, return_queue))
            return return_queue.get()

        return inner

    def _process_pending_calls(self):
        """Process all pending function calls to functions wrapped with queue_abstract"""
        while True:
            try:
                method, args, return_queue = self._calls_queue.get(block=False)
                return_queue.put(method(*args))
            except queue.Empty:
                break


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