"""Applicaiton configuration."""

from dynaconf import Dynaconf, Validator, ValidationError

from . import persistent

settings = Dynaconf(
    envvar_prefix="STARTRACKER",
    settings_files=[
        persistent.Persistent.get_instance().default_conf_file,
        persistent.Persistent.get_instance().conf_file,
    ],
    validators=[
        Validator("serial_port", must_exist=True),
        Validator("serial_baudrate", cast=int, must_exist=True),
        Validator("hardware.motor_dists_from_center", cast=float, must_exist=True),
        Validator("hardware.motor_theta", cast=float, must_exist=True),
        Validator("hardware.motor_phi", cast=float, must_exist=True),
        Validator("trajectory.max_seconds", cast=float, must_exist=True),
        Validator("trajectory.max_dist", cast=float, must_exist=True),
    ],
)


class ConfigurationException(Exception):
    """Errors concerning the configuration."""

    pass


def check_validation_errors():
    """Check and validate all configuration."""
    try:
        settings.validators.validate_all()
    except ValidationError as e:
        raise ConfigurationException(f"Configuration validation error: {e}")
    except Exception as e:
        raise ConfigurationException(f"Configuration error: {e}")
