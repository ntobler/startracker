"""Attitude estimation module tests."""

import numpy as np
import pytest
import scipy.spatial.transform

from startracker import attitude_estimation, calibration, testing_utils


def test_attutude_estimation_simple():
    cal = calibration.make_dummy()
    ae_conf = attitude_estimation.AttitudeEstimatorConfig(
        n_match=14, star_match_pixel_tol=1.0, timeout_secs=10
    )
    ae = attitude_estimation.AttitudeEstimator(cal, config=ae_conf)

    sig_conf = testing_utils.StarImageGeneratorConfig(noise_sigma=0)
    sig = testing_utils.StarImageGenerator(cal, sig_conf)

    rng = np.random.default_rng(42)
    quat_gt = scipy.spatial.transform.Rotation.random(random_state=rng).as_quat(canonical=False)

    image, _, _ = sig.image_from_quaternion(quat_gt)

    att_res = ae(image)

    np.testing.assert_allclose(att_res.quat, quat_gt, rtol=1e-3, atol=1e-3)


def test_attutude_estimation_error():
    cal = calibration.make_dummy()
    ae_conf = attitude_estimation.AttitudeEstimatorConfig(n_match=12)
    ae = attitude_estimation.AttitudeEstimator(cal, config=ae_conf)

    # Get very noisy image
    sig_conf = testing_utils.StarImageGeneratorConfig(noise_sigma=200)
    sig = testing_utils.StarImageGenerator(cal, sig_conf)

    image, _, _ = sig([0, 0, 1], [0, 1, 0])

    att_res = ae(image)
    assert att_res == attitude_estimation.ERROR_ATTITUDE_RESULT


def test_attutude_estimation():
    rng = np.random.default_rng(42)

    cal = calibration.make_dummy()
    ae_conf = attitude_estimation.AttitudeEstimatorConfig(n_match=12)
    ae = attitude_estimation.AttitudeEstimator(cal, config=ae_conf)
    sig = testing_utils.StarImageGenerator(cal)

    vectors = rng.normal(size=(10, 3))
    vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)

    true_positive_mags = []
    false_negative_mags = []

    for _, vector in enumerate(vectors):
        image, gt_xy, mag = sig(vector, [0, 1, 0])

        att_res = ae(image)

        tp, fn = ae.calculate_statistics(att_res)
        true_positive_mags.extend(tp)
        false_negative_mags.extend(fn)

        image_xy = (sig.intrinsic @ att_res.image_xyz_cam.T).T
        image_xy = image_xy[..., :2] / image_xy[..., 2:]
        if sig.distorter is not None:
            image_xy = sig.distorter.distort(image_xy)

        cat_xy = (sig.intrinsic @ att_res.cat_xyz_cam.T).T
        cat_xy = cat_xy[..., :2] / cat_xy[..., 2:]
        if sig.distorter is not None:
            cat_xy = sig.distorter.distort(cat_xy)

        rot = scipy.spatial.transform.Rotation.from_quat(att_res.quat)
        res = rot.apply([0, 0, 1])

        delta_angle = np.degrees(np.arccos(np.dot(vector, res)))

        assert delta_angle < 0.1

        if False:
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, constrained_layout=False)
            ax.imshow(image)
            ax.plot(gt_xy[..., 0], gt_xy[..., 1], "x")
            ax.plot(image_xy[..., 0], image_xy[..., 1], "x")
            ax.plot(cat_xy[..., 0], cat_xy[..., 1], "x")
            for (x, y), m in zip(gt_xy, mag):
                ax.text(x, y, f" {m:.1f}")
            fig.suptitle(
                f"quat: {att_res.quat}, n_matches: {att_res.n_matches}, "
                f"delta_angle: {delta_angle:.3f}"
            )
            plt.show()

    # Calculate histograms over magnitudes for true positives and false negatives
    min_mag = 4
    max_mag = 5.5
    bins = np.linspace(min_mag, max_mag, int((max_mag - min_mag) * 10) + 1)
    tp_hist = np.histogram(true_positive_mags, bins=bins)[0]
    fn_hist = np.histogram(false_negative_mags, bins=bins)[0]

    f1 = 2 * tp_hist / (2 * tp_hist + fn_hist)
    cumulative_f1 = 2 * np.cumsum(tp_hist) / np.cumsum(2 * tp_hist + fn_hist)

    assert np.all(cumulative_f1 > 0.8)

    if False:
        import matplotlib.pyplot as plt

        bin_centers = (bins[:-1] + bins[1:]) * 0.5

        fig, axs = plt.subplots(2, constrained_layout=True, sharex=True)
        axs[0].hist(true_positive_mags, bins=bins, alpha=0.5, label="true positives")
        axs[0].hist(false_negative_mags, bins=bins, alpha=0.5, label="false negatives")
        axs[0].legend()
        axs[1].plot(bin_centers, f1, label="F1 score")
        axs[1].plot(bin_centers, cumulative_f1, label="Cumulative F1 score")
        axs[1].set_xlabel("Star magnitude")
        axs[1].legend()
        fig.suptitle("Histogram of detected magnitudes")
        plt.show()


if __name__ == "__main__":
    pytest.main([__file__])
