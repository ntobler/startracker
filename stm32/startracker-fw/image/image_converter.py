"""Script to convert png images to the display binary format."""

import math
import pathlib

import cv2
import numpy as np


def generate_bit_image(image_file: pathlib.Path, h_file: pathlib.Path, name: str):
    image = cv2.imread(str(image_file), cv2.IMREAD_GRAYSCALE)

    shape = image.shape
    height, width = shape

    newshape = (math.ceil(shape[0] / 8) * 8, shape[1])

    bigger = np.zeros(newshape, np.uint8)
    bigger[: shape[0]] = image

    res = np.packbits(bigger, axis=0, bitorder="little")

    data = ",".join([str(x) for x in res.ravel()])

    filecontent = f"""\
#include "image.h"

static const uint8_t {name}_data[] ATTR_TO_FLASH = {{{data}}};

Image_description_t {name} = {{
    .width = {width},
    .height = {height},
    .data = {name}_data
}};
"""

    with h_file.open("w") as f:
        f.write(filecontent)


def scan_folder(path: pathlib.Path):
    for file in path.glob("*.png"):
        print(file)
        outfile = file.parent / (file.stem + ".c")
        name = file.stem
        generate_bit_image(file, outfile, name)


if __name__ == "__main__":
    scan_folder(pathlib.Path(__file__).parent.absolute().resolve())
