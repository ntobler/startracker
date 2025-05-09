"""Build project. Currently this downloads vue.js."""

import pathlib
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
    try:
        urllib.request.urlretrieve(vue_url, output_file)  # noqa: S310
        print(f"Vue.js has been downloaded and saved as {output_file}")
    except Exception as e:
        print(f"Failed to download Vue.js. Error: {e}")


if __name__ == "__main__":
    download_vue()
