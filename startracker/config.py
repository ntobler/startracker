"""Application configuration."""

from dynaconf import Dynaconf, ValidationError, Validator

from startracker import persistent

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
        Validator("shutdown_delay", cast=float, must_exist=True),
    ],
)


class ConfigurationError(Exception):
    """Errors concerning the configuration."""


def check_validation_errors():
    """Check and validate all configuration."""
    try:
        settings.validators.validate_all()
    except ValidationError as e:
        raise ConfigurationError(f"Configuration validation error: {e}") from e
    except Exception as e:
        raise ConfigurationError(f"Configuration error: {e}") from e
