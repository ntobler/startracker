"""Protocol definitions for the UART connection to the STM32 main board."""


def set_settings(
    min_matches: int,
    exposure_ms: int,
    gain: int,
    attitude_estimation_timeout_ms: int,
):
    return {
        "acknowledge": (True, False),
    }


class Mode:
    OFF = 0
    ON = 1
    SINGLE = 2


def set_attitude_estimation_mode(
    mode: Mode,
):
    return {
        "acknowledge": True,
    }


def record_dark_frame():
    return {
        "acknowledge": True,
    }


def get_status():
    return {
        "attitude estimation": ("on", "off"),
        "current_number_of_matches": int,
        "average_number_of_matches": int,
        "quaternion": [float, float, float, float],
        "estimation_id": int,
    }


def shutdown():
    return {
        "acknowledge": True,
    }
