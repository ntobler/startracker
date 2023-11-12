"""Script to convert png images to the display binary format."""

import pathlib

import cv2
import math
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
#include "stdint.h"
#include "image.h"

static uint8_t {name}_symbol_data[] = {{{data}}};

Image_description_t {name}_symbol = {{
    .width = {width},
    .height = {height},
    .data = {name}_symbol_data
}};
"""

    with open(h_file, "w") as f:
        f.write(filecontent)


def scan_folder(path: pathlib.Path):
    for file in path.glob("*.png"):
        print(file)
        outfile = file.parent / (file.stem + ".c")
        name = file.stem
        generate_bit_image(file, outfile, name)


if __name__ == "__main__":
    scan_folder(pathlib.Path(__file__).parent.absolute().resolve())
