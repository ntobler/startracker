"""Temporal filter to estimate bias and scale coefficients of the gyroscope.

Based on periodic absolute attitude measurements.
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import scipy.spatial

from startracker import libstartracker

SCALE_RAD_S_PER_DIGIT = (500.0 * np.pi) / ((1 << 15) * 180.0)


def main():
    """Run test script to showcase the filter is working as intended."""
    rng = np.random.default_rng(42 + 2)

    n = 200
    fs = 200

    scale = rng.uniform(low=-1, high=1, size=(3,)) * 0.01  # +-1% assumed
    bias = rng.uniform(low=-1, high=1, size=(3,)) * np.radians(
        1
    )  # +-1 degree per second per datasheet

    n_blocks = 200

    filter_length = 64

    omega_t = rng.normal(size=(n * n_blocks, 3)).astype(np.float64)
    omega_t = np.stack(
        [
            scipy.signal.convolve(
                omega_t[..., i], np.ones((filter_length,)) / np.sqrt(filter_length), mode="same"
            )
            for i in range(omega_t.shape[1])
        ],
        axis=-1,
    )

    omega_t_blocks = omega_t.reshape((n_blocks, n, 3))

    if True:
        amplitude = np.logspace(np.log10(0.001), np.log10(5), n_blocks)
        omega_t_blocks *= amplitude[..., np.newaxis, np.newaxis]
    if False:
        amplitude = 10 ** (rng.uniform(np.log10(0.001), np.log10(1), size=n_blocks))
        omega_t_blocks *= amplitude[..., np.newaxis, np.newaxis]
    if False:
        amplitude = 1
        omega_t_blocks *= amplitude

    snr_blocks = []
    omega_t_measured_blocks = []
    q_1_blocks = []
    scale_est = np.array((0.0, 0.0, 0.0))
    bias_est = np.array((0.0, 0.0, 0.0))

    all_scale_est = []
    all_bias_est = []
    all_mag = []
    all_loss = []
    all_snr = []

    q_0 = np.array((1.0, 0.0, 0.0, 0.0)).tolist()
    for omega_t in omega_t_blocks:
        loss, _, q_1 = libstartracker.estimate_gyro_params(
            q_0,
            q_0,
            omega_t,
            [0, 0, 0],
            [0, 0, 0],
            1.0 / fs,
        )
        omega_t_transformed = (omega_t - bias) / (1.0 + scale)
        omega_t_measured = omega_t_transformed + rng.normal(
            size=omega_t_transformed.shape
        ) * np.radians(0.007) * np.sqrt(fs)
        omega_t_measured = (
            np.clip(omega_t_measured / SCALE_RAD_S_PER_DIGIT, -(2**15), 2**15 - 1).round()
            * SCALE_RAD_S_PER_DIGIT
        )

        s = np.std(omega_t_transformed)
        n = np.std(omega_t_measured - omega_t_transformed)
        snr_db = np.log10(s / n) * 20.0

        snr_blocks.append(snr_db)
        omega_t_measured_blocks.append(omega_t_measured)
        q_1_blocks.append(q_1)
        q_0 = q_1

    omega_t_measured_blocks = np.array(omega_t_measured_blocks)

    _, axs = plt.subplots(3)
    axs[0].plot(omega_t_blocks.reshape((-1, 3)))
    axs[1].plot(omega_t_measured_blocks.reshape((-1, 3)))
    axs[2].plot(snr_blocks)
    plt.show()

    q_0 = np.array((1.0, 0.0, 0.0, 0.0)).tolist()
    for omega_t_measured, q_1, snr in zip(omega_t_measured_blocks, q_1_blocks, snr_blocks):
        mag = (
            scipy.spatial.transform.Rotation.from_quat(q_0, scalar_first=True)
            * scipy.spatial.transform.Rotation.from_quat(q_1, scalar_first=True).inv()
        ).magnitude()

        scale_est, bias_est = libstartracker.filter_step(
            q_0, q_1, omega_t_measured, scale_est, bias_est, 1 / fs, 1e-3, 0.2, 1e-3
        )

        all_scale_est.append(scale_est)
        all_bias_est.append(bias_est)
        all_mag.append(mag)
        all_loss.append(loss)
        all_snr.append(snr)

        q_0 = q_1

    all_scale_est = np.array(all_scale_est)
    all_bias_est = np.array(all_bias_est)
    all_mag = np.array(all_mag)
    all_loss = np.array(all_loss)
    all_snr = np.array(all_snr)

    x = len(all_bias_est) - 1

    _, axs = plt.subplots(5, sharex=True)
    axs[0].plot(all_scale_est)
    axs[0].set_prop_cycle(None)
    axs[0].plot([0, x], [scale, scale], "-", alpha=0.3)
    axs[0].set_ylabel("scale")

    axs[1].plot(all_bias_est)
    axs[1].set_prop_cycle(None)
    axs[1].plot([0, x], [bias, bias], "-", alpha=0.3)
    axs[1].set_ylabel("bias")

    axs[2].plot(all_loss)
    axs[2].set_ylabel("loss")

    axs[3].plot(all_mag)
    axs[3].set_ylabel("quaternion_differnce")

    axs[4].plot(all_snr)
    axs[4].set_ylabel("snr")

    axs[4].set_xlabel("sample")
    plt.show()


if __name__ == "__main__":
    main()
