"""Build project. Currently this downloads vue.js."""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys
import urllib.request

vue_url = "https://cdnjs.cloudflare.com/ajax/libs/vue/3.5.4/vue.esm-browser.prod.min.js"
"""URL for the Vue.js production build"""

project_path = pathlib.Path(__file__).parent.expanduser().absolute()
"""Path pointing to the project root."""

output_file = project_path / "web/vue.esm-browser.prod.min.js"
"""Output file path"""


def download_vue() -> None:
    """Download the `vue.js` library from a CDN to use host locally."""
    print(f"Downloading Vue.js from {vue_url}...")
    if output_file.is_file():
        print("Vue.js has already been downloaded.")
        return
    try:
        urllib.request.urlretrieve(vue_url, output_file)  # noqa: S310
        print(f"Vue.js has been downloaded and saved as {output_file}")
    except Exception as e:
        print(f"Failed to download Vue.js. Error: {e}")


def rust_build(*, debug: bool) -> None:
    """Build rust backend and move shared library to correct folder."""
    cwd = pathlib.Path(__file__).parent.expanduser().absolute()
    if debug:
        env = {"CARGO_TARGET_DIR": str(cwd / "target"), "PYO3_PYTHON": sys.executable}
        env.update(os.environ)
        subprocess.check_call(["cargo", "build"], cwd=cwd, env=env)  # noqa: S603, S607
    else:
        subprocess.check_call(["cargo", "build", "--release"], cwd=cwd)  # noqa: S603, S607
    shutil.copy(cwd / "target/release/libstartracker.so", cwd / "startracker/libstartracker.so")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build the StarTracker project.")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Build in debug mode (default: release mode).",
    )
    args = parser.parse_args()

    download_vue()
    rust_build(debug=args.debug)
