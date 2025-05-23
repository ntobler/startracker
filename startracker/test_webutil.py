"""Util module tests."""

import threading
import time

import pytest

from startracker import webutil


def test_data_dispatcher():
    image_data = webutil.DataDispatcher()

    n_threads = 4

    received = [[] for _ in range(n_threads)]
    terminate = False

    def func(thread_id):
        while not terminate:
            received[thread_id].append(image_data.get_blocking())

    threads = []
    for i in range(n_threads):
        t = threading.Thread(target=func, args=(i,))
        t.start()
        threads.append(t)

    data = [1, "sdf", 4.5, b"sdfasd"]

    for d in data[:-1]:
        image_data.put(d)
        time.sleep(0.1)
    terminate = True
    image_data.put(data[-1])

    for t in threads:
        t.join()

    for rec in received:
        assert rec == data


if __name__ == "__main__":
    pytest.main([__file__])
