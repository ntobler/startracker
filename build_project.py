"""Build project. Currently this downloads vue.js."""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys
import urllib.request

urls = {
    "vue.esm-browser.prod.min.js": "https://cdnjs.cloudflare.com/ajax/libs/vue/3.5.4/vue.esm-browser.prod.min.js",
    "katex.mjs": "https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.mjs",
    "katex.min.css": "https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css",
}
"""URL for JavaScript dependency production builds."""

project_path = pathlib.Path(__file__).parent.expanduser().absolute()
"""Path pointing to the project root."""


def download_js_dependencies() -> None:
    """Download the `vue.js` library from a CDN to use host locally."""
    for name, url in urls.items():
        output_file = project_path / "web" / name
        if output_file.is_file():
            print(f"{name} has already been downloaded.")
            continue
        try:
            print(f"Downloading {name} from {url} to {output_file}")
            urllib.request.urlretrieve(url, output_file)  # noqa: S310
        except Exception as e:
            print(f"Failed to download {name}. Error: {e}")


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

    download_js_dependencies()
    rust_build(debug=args.debug)
