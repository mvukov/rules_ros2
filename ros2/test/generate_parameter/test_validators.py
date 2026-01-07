"""Custom parameter validators for testing."""

from typing import Any
from rclpy.parameter import Parameter


def string_not_empty(parameter: Parameter) -> None:
    """Validate that a string parameter is not empty.

    Args:
        parameter: The parameter to validate

    Raises:
        ValueError: If the parameter value is empty
    """
    param_value = parameter.value
    if not param_value:
        raise ValueError(f"Parameter '{parameter.name}' cannot be empty")


def positive_integer(parameter: Parameter) -> None:
    """Validate that an integer parameter is positive.

    Args:
        parameter: The parameter to validate

    Raises:
        ValueError: If the parameter value is not positive
    """
    param_value = parameter.value
    if param_value <= 0:
        raise ValueError(
            f"Parameter '{parameter.name}' must be positive, got {param_value}"
        )


def in_range(parameter: Parameter, min_value: float, max_value: float) -> None:
    """Validate that a parameter value is within a specified range.

    Args:
        parameter: The parameter to validate
        min_value: Minimum allowed value (inclusive)
        max_value: Maximum allowed value (inclusive)

    Raises:
        ValueError: If the parameter value is outside the specified range
    """
    param_value = parameter.value
    if param_value < min_value or param_value > max_value:
        raise ValueError(
            f"Parameter '{parameter.name}' value {param_value} is outside "
            f"range [{min_value}, {max_value}]"
        )
