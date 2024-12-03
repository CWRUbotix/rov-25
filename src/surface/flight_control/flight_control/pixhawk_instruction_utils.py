from collections.abc import Callable

from rov_msgs.msg import PixhawkInstruction


def pixhawk_instruction_to_tuple(
    msg: PixhawkInstruction,
) -> tuple[float, float, float, float, float, float]:
    """
    Convert PixhawkInstruction to tuple of dimensions.

    Parameters
    ----------
    msg : PixhawkInstruction
        PixhawkInstruction to convert

    Returns
    -------
    tuple[float, float, float, float, float, float]
        Tuple of dimensions from the instruction
    """
    return (msg.forward, msg.vertical, msg.lateral, msg.pitch, msg.yaw, msg.roll)


def tuple_to_pixhawk_instruction(
    instruction_tuple: tuple[float, ...], author: int = PixhawkInstruction.MANUAL_CONTROL
) -> PixhawkInstruction:
    """
    Convert tuple of dimensions and author to a PixhawkInstruction.

    Parameters
    ----------
    instruction_tuple : tuple[float, ...]
        Tuple of dimensions
    author : int, optional
        Author of the PixhawkInstruction, by default PixhawkInstruction.MANUAL_CONTROL

    Returns
    -------
    PixhawkInstruction
        A new PixhawkInstruction with the provided dimensions and author
    """
    return PixhawkInstruction(
        forward=instruction_tuple[0],
        lateral=instruction_tuple[1],
        vertical=instruction_tuple[2],
        roll=instruction_tuple[3],
        pitch=instruction_tuple[4],
        yaw=instruction_tuple[5],
        author=author,
    )


def apply_function(
    msg: PixhawkInstruction, function_to_apply: Callable[[float], float]
) -> PixhawkInstruction:
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
    instruction_tuple = pixhawk_instruction_to_tuple(msg)
    modified_tuple = tuple(function_to_apply(value) for value in instruction_tuple)
    return tuple_to_pixhawk_instruction(modified_tuple, msg.author)
