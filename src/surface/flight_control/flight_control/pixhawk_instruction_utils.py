from typing import TypeAlias

from rov_msgs.msg import PixhawkInstruction

InstructionTuple: TypeAlias = tuple[float, float, float, float, float, float]


def pixhawk_instruction_to_tuple(msg: PixhawkInstruction) -> InstructionTuple:
    return (msg.forward, msg.vertical, msg.lateral, msg.pitch, msg.yaw, msg.roll)


def tuple_to_pixhawk_instruction(
    instruction_tuple: InstructionTuple, author: int = PixhawkInstruction.MANUAL_CONTROL
) -> PixhawkInstruction:
    return PixhawkInstruction(
        forward=instruction_tuple[0],
        lateral=instruction_tuple[1],
        vertical=instruction_tuple[2],
        roll=instruction_tuple[3],
        pitch=instruction_tuple[4],
        yaw=instruction_tuple[5],
        author=author,
    )
