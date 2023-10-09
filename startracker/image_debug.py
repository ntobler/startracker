"""
For debug purposes.
"""

# %%

import pathlib

import numpy as np
import subprocess
import matplotlib.pyplot as plt

import image_processing

# Copy code to Raspberry pi
subprocess.run(["bash", "copy.sh"], cwd="../install")

# %%

directory = pathlib.Path("../test_images2")

data = np.load(directory / "lightframe.npy")
print(data.shape, data.dtype, data.max(), data.min())

data = image_processing.decode_sbggr12_1x12(data)

def plot_channels(x):
    fig, axs = plt.subplots(2, 2, sharex=True, sharey=True)
    for i in range(2):
        for j in range(2):
            axs[i, j].imshow(x[i::2, j::2])
    fig.show()

plot_channels(data)

# Only use green channels to circumvent chromatic abberation
data = image_processing.extract_green(data)
data -= 250
data = data.clip(0, 255).astype(np.uint8)

data = image_processing.binning(data, factor=2)

plt.figure()
plt.imshow(data)
plt.show()
