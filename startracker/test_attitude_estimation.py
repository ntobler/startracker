import numpy as np
import scipy.spatial.transform
import matplotlib.pyplot as plt
import cots_star_tracker

from . import attitude_estimation
from . import testing_utils


def test_attutude_estimation():
    tm = testing_utils.TestingMaterial(use_existing=True)

    rng = np.random.default_rng(42)

    ae = attitude_estimation.AttitudeEstimator(tm.cam_file, tm.stardata_dir)
    intrinsic, (width, height), dist_coeffs = cots_star_tracker.read_cam_json(
        tm.cam_file
    )

    sig = testing_utils.StarImageGenerator(intrinsic, (width, height), dist_coeffs)

    vectors = rng.normal(size=(100, 3))
    vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)

    for vector in vectors:
        image, gt_xy, mag = sig(vector, [0, 1, 0])

        quat, n_matches, image_xyz, cat_xyz = ae(image)

        image_xy = (sig.intrinsic @ image_xyz.T).T
        image_xy = image_xy[..., :2] / image_xy[..., 2:]
        if sig.distorter is not None:
            image_xy = sig.distorter.distort(image_xy)

        cat_xy = (sig.intrinsic @ cat_xyz.T).T
        cat_xy = cat_xy[..., :2] / cat_xy[..., 2:]
        if sig.distorter is not None:
            cat_xy = sig.distorter.distort(cat_xy)

        rot = scipy.spatial.transform.Rotation.from_quat(quat)
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
                f"quat: {quat}, n_matches: {n_matches}, delta_angle: {delta_angle:.3f}"
            )
            plt.show()
