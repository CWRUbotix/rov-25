from collections.abc import Callable

from mavros_msgs.msg import ManualControl


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


def tuple_to_manual_control(
    instruction_tuple: tuple[float, ...]
) -> ManualControl:
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
    Run the provided function on each dimension of a PixhawkInstruction.
    Does not modify the original PixhawkInstruction.

    Parameters
    ----------
    msg : PixhawkInstruction
        The instruction to run the function on
    function_to_apply : Callable[[float], float]
        The function to apply to each dimension

    Returns
    -------
    PixhawkInstruction
        The new PixhawkInstruction made by applying the function to each dimension of msg
    """
    instruction_tuple = manual_control_to_tuple(msg)
    modified_tuple = tuple(function_to_apply(value) for value in instruction_tuple)
    return tuple_to_manual_control(modified_tuple)
