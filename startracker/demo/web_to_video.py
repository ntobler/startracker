"""Automated demo video generation."""

import io
import pathlib
import time

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright

from startracker import util
from startracker.demo import demo_utils


def make_demo_video():
    """Create vidio of the web_gui."""
    vc = demo_utils.VideoCreator()

    def snap(page, n: int = 1, frame_rate: float | None = None):
        """Capture screenshots of the page and add to the video."""
        for i in range(n):
            with util.max_rate(frame_rate or 1000):
                print(f"snapping {i} of {n}, {frame_rate=}")
                png_bytes = page.screenshot(full_page=True)
                pil_image = Image.open(io.BytesIO(png_bytes))
                frame = np.array(pil_image)

                if frame_rate is None:
                    for _ in range(n):
                        vc.push(frame)
                else:
                    vc.push(frame)
            if frame_rate is None:
                break

    frame_rate = 4

    with sync_playwright() as p:
        browser = p.chromium.launch()
        context = browser.new_context(
            viewport={"width": 960, "height": 500},
            # user_agent="Mozilla/5.0 (iPhone; CPU iPhone OS 14_0 like Mac OS X)...",
            is_mobile=True,
            device_scale_factor=2,
            has_touch=True,
        )
        page = context.new_page()
        page.goto("http://localhost:5000")
        snap(page, n=4, frame_rate=frame_rate)
        page.click("#toggle_cam")
        snap(page, n=20, frame_rate=frame_rate)
        page.goto("http://localhost:5000/axisAlign.html")
        time.sleep(1)
        snap(page, n=22, frame_rate=frame_rate)
        page.click("#toggle_cam")
        snap(page, n=6, frame_rate=frame_rate)

        browser.close()

    vc.save_webp(pathlib.Path("./startracker.webp"), frame_rate=frame_rate)
    print("saved")


if __name__ == "__main__":
    make_demo_video()
