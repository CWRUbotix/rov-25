from collections.abc import Callable
from typing import Final

from mavros_msgs.msg import ManualControl

# Brown out protection
SPEED_THROTTLE: Final = 0.65

# Joystick curve
JOYSTICK_EXPONENT: Final = 3

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: Final = 0
Z_ZERO_SPEED: Final = 500
MAX_RANGE_SPEED: Final = 2000
Z_MAX_RANGE_SPEED: Final = 1000
RANGE_SPEED: Final = MAX_RANGE_SPEED * SPEED_THROTTLE
Z_RANGE_SPEED: Final = Z_MAX_RANGE_SPEED * SPEED_THROTTLE


def joystick_map(raw: float) -> float:
    """
    Convert the provided joystick position to a
    float in [-1300, 1300] for use in a ManualControl instruction.

    Parameters
    ----------
    raw : float
        The joystick position to convert

    Returns
    -------
    float
        A float in [-1300, 1300] to act as a ManualControl dimension
    """
    mapped = abs(raw) ** JOYSTICK_EXPONENT
    if raw < 0:
        mapped *= -1
    mapped = RANGE_SPEED * mapped + ZERO_SPEED
    return mapped


def manual_control_to_tuple(
    msg: ManualControl,
) -> tuple[float, float, float, float, float, float]:
    """
    Convert ManualControl to tuple of dimensions.

    Parameters
    ----------
    msg : ManualControl
        ManualControl to convert

    Returns
    -------
    tuple[float, float, float, float, float, float]
        Tuple of dimensions from the instruction
    """
    return (msg.x, msg.z, msg.y, msg.s, msg.r, msg.t)


def tuple_to_manual_control(instruction_tuple: tuple[float, ...]) -> ManualControl:
    """
    Convert tuple of dimensions and author to a ManualControl.

    Parameters
    ----------
    instruction_tuple : tuple[float, ...]
        Tuple of dimensions

    Returns
    -------
    ManualControl
        A new ManualControl with the provided dimensions and author
    """
    return ManualControl(
        x=instruction_tuple[0],
        z=instruction_tuple[1],
        y=instruction_tuple[2],
        s=instruction_tuple[3],
        r=instruction_tuple[4],
        t=instruction_tuple[5],
    )


def apply_function(
    msg: ManualControl, function_to_apply: Callable[[float], float]
) -> ManualControl:
    """
    Run the provided function on each dimension of a ManualControl instruction.
    Does not modify the original ManualControl.

    Parameters
    ----------
    msg : ManualControl
        The instruction to run the function on
    function_to_apply : Callable[[float], float]
        The function to apply to each dimension

    Returns
    -------
    ManualControl
        The new ManualControl made by applying the function to each dimension of msg
    """
    instruction_tuple = manual_control_to_tuple(msg)
    modified_tuple = tuple(function_to_apply(value) for value in instruction_tuple)
    return tuple_to_manual_control(modified_tuple)
