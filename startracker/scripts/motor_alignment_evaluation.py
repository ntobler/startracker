"""Test script for motor alignment algorithms."""
# TODO clean up

# %%
# Imports

import matplotlib.pyplot as plt
import numpy as np
import scipy.spatial

from startracker import trajectory

# %%
# Polynom trajectory


def plot_polynom_trajectory():
    """Plot a polynomial trajectory example."""
    polytr = trajectory.TrajectoryCalculator(15 * 60, 3, trajectory.MotorSolver())(35, 47)

    times = np.linspace(polytr.start, polytr.stop, 1000)
    path_approx = [np.polyval(coeffs, times) for coeffs in polytr.position_coeffs]
    path_approx = np.array(path_approx).T

    plt.figure()
    plt.plot(times, path_approx)
    plt.show()


plot_polynom_trajectory()

# %%


def calc_motor_dists(azimuths, elevations, rolls, **kwargs):
    m = trajectory.MotorSolver(**kwargs)
    rots = []
    for a, e, r in zip(azimuths.ravel(), elevations.ravel(), rolls.ravel()):
        rots.append(trajectory.astro_rotation_matrix(float(a), float(e), float(r), degrees=True))
    rots = np.array(rots)
    rots = rots.reshape(azimuths.shape + rots.shape[1:])
    ret = m.solve_motor_dists(rots, plot=False)
    return ret


def polar_plot(elevations, azimuths, values):
    fig, axes = plt.subplots(figsize=(9, 7), subplot_kw={"projection": "polar"})
    contourplot = axes.contourf(np.radians(azimuths), 90 - elevations, values)
    plt.colorbar(contourplot, shrink=0.6, pad=0.08)
    plt.show()


def el_az_plot(elevations, azimuths, values):
    fig, axes = plt.subplots(figsize=(9, 5))
    contourplot = axes.contourf(azimuths, elevations, values)
    plt.colorbar(contourplot, shrink=0.6, pad=0.08)
    plt.xlabel("Azimuth (degrees)")
    plt.ylabel("Elevation (degrees)")
    plt.grid()
    plt.show()


def max_motor_dists():
    azimuths, elevations, rolls = np.meshgrid(
        np.arange(0, 181, 2), np.arange(1, 90, 2), [-1.875, 1.875], indexing="ij"
    )
    motor_dists = calc_motor_dists(azimuths, elevations, rolls, theta=np.radians(90))

    plt.figure(figsize=(9, 7))
    for j in range(2):
        for i in range(3):
            plt.plot(elevations[..., j].T, motor_dists[..., j, i].T)
    plt.xlabel("Elevation (deg)")
    plt.ylabel("Maximum motor distance (mm)")
    plt.grid()
    plt.show()

    polar_plot(
        elevations[..., 0],
        azimuths[..., 0],
        np.max(motor_dists, axis=(-1, -2)) - np.min(motor_dists, axis=(-1, -2)),
    )
    el_az_plot(
        elevations[..., 0],
        azimuths[..., 0],
        np.max(motor_dists, axis=(-1, -2)) - np.min(motor_dists, axis=(-1, -2)),
    )


max_motor_dists()


def max_stepper_error(step_size=2.5e-3):
    """Plot histogram of error, if the stepper has discrete steps of a given step size."""
    azimuths, elevations, rolls = np.meshgrid(
        np.arange(0, 121, 2), np.arange(1, 90, 2), [-1.875, 1.875], indexing="ij"
    )
    motor_dists = calc_motor_dists(azimuths, elevations, rolls, theta=np.radians(90))

    m = trajectory.MotorSolver(theta=np.radians(90))
    rots = []
    for a, e, r in zip(azimuths.ravel(), elevations.ravel(), rolls.ravel()):
        rots.append(trajectory.astro_rotation_matrix(float(a), float(e), float(r), degrees=True))
    rots = np.array(rots)
    rots = rots.reshape(azimuths.shape + rots.shape[1:])
    motor_dists = m.solve_motor_dists(rots, plot=False)
    motor_dists = np.round(motor_dists / step_size) * step_size
    effective_rots = m.get_rotm(motor_dists)

    rots = rots.reshape((-1, 3, 3))
    effective_rots = effective_rots.reshape((-1, 3, 3))
    errors = scipy.spatial.transform.Rotation.from_matrix(
        rots @ np.swapaxes(effective_rots, -1, -2)
    ).magnitude()

    plt.figure(figsize=(9, 7))
    plt.hist(trajectory.degrees_to_seconds(np.degrees(errors)), bins=20)
    plt.xlabel("Error (seconds)")
    plt.yticks([])
    plt.title(f"Error distribution assuming stepper step of {step_size*1000}um")
    plt.grid()
    plt.show()


max_stepper_error()


def motor_orientation_test(theta_range, phi_range, roll_angle=1.875):
    thetas, phis = np.meshgrid(theta_range, phi_range, indexing="ij")

    max_motor_ranges = np.zeros_like(thetas, dtype=np.float64)
    min_motor_ranges = np.zeros_like(thetas, dtype=np.float64)
    for i, (t, p) in enumerate(zip(thetas.ravel(), phis.ravel())):
        azimuths, elevations, rolls = np.meshgrid(
            np.arange(0, 121, 2),
            np.arange(1, 90, 2),
            [-roll_angle, roll_angle],
            indexing="ij",
        )
        motor_dists = calc_motor_dists(
            azimuths, elevations, rolls, theta=np.radians(t), phi=np.radians(p)
        )

        motor_ranges = np.max(motor_dists, axis=(-1, -2)) - np.min(motor_dists, axis=(-1, -2))

        a, b = np.unravel_index(i, thetas.shape)
        max_motor_ranges[a, b] = np.max(motor_ranges)
        min_motor_ranges[a, b] = np.mean(np.abs(motor_dists[..., 1, :] - motor_dists[..., 0, :]))
    max_motor_ranges = np.clip(max_motor_ranges, None, 20)
    min_motor_ranges = np.clip(min_motor_ranges, None, 20)

    fig, axs = plt.subplots(2, figsize=(9, 7), sharex=True, sharey=True)
    contourplot = axs[0].contourf(thetas, phis, max_motor_ranges)
    cbar = plt.colorbar(contourplot, shrink=0.6, pad=0.08)
    cbar.ax.set_ylabel("Millimeters")
    axs[0].grid()

    mins = roll_angle * 2 / 360 * 24 * 60
    axs[0].set_xlabel("Motor shaft azimuth (degrees)")
    axs[0].set_ylabel("Motor shaft elevation (degrees)")
    axs[0].set_title(f"max motor range needed to travel +-{roll_angle} degrees ({mins}mins)")

    contourplot = axs[1].contourf(thetas, phis, min_motor_ranges)
    cbar = plt.colorbar(contourplot, shrink=0.6, pad=0.08)
    cbar.ax.set_ylabel("Millimeters")
    axs[1].grid()
    axs[1].set_xlabel("Motor shaft azimuth (degrees)")
    axs[1].set_ylabel("Motor shaft elevation (degrees)")
    axs[1].set_title(f"Average motor range traveled for +-{roll_angle} degrees ({mins}mins)")
    fig.tight_layout()
    plt.show()


low_res = 2
theta_range = np.arange(45, 136, 5.0 * low_res)
phi_range = np.arange(10, 91, 10 * low_res)
motor_orientation_test(theta_range, phi_range)

# %%

theta_range = np.arange(75, 106, 2.5)
phi_range = np.arange(45, 71, 5)
motor_orientation_test(theta_range, phi_range)

# %%
azimuths, elevations, rolls = np.meshgrid(
    np.arange(1), np.arange(0, 91, 10), np.arange(-5, 5, 0.2), indexing="ij"
)
motor_dists = calc_motor_dists(azimuths, elevations, rolls)


plt.figure()
# plt.plot(rolls[0].T, np.max(motor_dists[0], axis=-1).T, color="red")
# plt.plot(rolls[0].T, np.min(motor_dists[0], axis=-1).T, color="blue")
for i in range(3):
    plt.plot(rolls[0].T, motor_dists[0, ..., i].T, ".-", color="red")
plt.show()

# %%
# Test polynomial fitting

path = motor_dists[0, 0, :, 0]
roll = rolls[0, 0, :]

poly = np.polyfit(roll, path, 3)
path_approx = np.polyval(poly, roll)

plt.figure()
plt.plot(roll, path, ".", color="red")
plt.plot(roll, path_approx, "-")
plt.show()

# %%
azimuths, elevations = np.meshgrid(np.arange(0, 121, 10), np.arange(0, 91, 10), indexing="ij")
rolls = np.ones_like(azimuths) * 5
motor_dists = calc_motor_dists(azimuths, elevations, rolls)


fig, axs = plt.subplots(2, 6, figsize=(9, 7), sharex=True, sharey=True)
for a_index, ax in enumerate(axs.flatten()):
    for i in range(3):
        ax.plot(elevations[a_index], motor_dists[a_index, ..., i], color="red")
    ax.set_title(f"Azimuth={azimuths[a_index, 0]:.2f}")
plt.show()
