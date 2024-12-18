from collections import deque
from queue import Queue
from typing import TypeVar

import pytest
from transceiver.serial_reader import SerialReaderPacketHandler

from rov_msgs.msg import FloatData

T = TypeVar('T')

PACKET = 'ROS:11,1,1:313901,988.53;314843,988.57;315785,988.99;316727,991.56;317669,996.21;318611,1002.36;319553,1010.36;320495,1021.11;321437,1034.42;322379,1050.23;323321,1051.86;324263,1053.20;325206,1053.32;326146,1053.46;327088,1053.52;328030,1053.58;328972,1053.61;329914,1053.64;330856,1053.61;331798,1053.60;332740,1053.65;333682,1053.58;334624,1053.53;335566,1053.52;336508,1053.39;337453,1053.41;338395,1053.46;339337,1053.37;340279,1053.42;341221,1053.49;342163,1053.54'  # noqa: E501

NOT_THREE_SECTIONS = 'ROS:11,1,1313901,988.53;314843,988.57;315785,988.99;316727,991.56;317669,996.21;318611,1002.36;319553,1010.36;320495,1021.11;321437,1034.42;322379,1050.23;323321,1051.86;324263,1053.20;325206,1053.32;326146,1053.46;327088,1053.52;328030,1053.58;328972,1053.61;329914,1053.64;330856,1053.61;331798,1053.60;332740,1053.65;333682,1053.58;334624,1053.53;335566,1053.52;336508,1053.39;337453,1053.41;338395,1053.46;339337,1053.37;340279,1053.42;341221,1053.49;342163,1053.54'  # noqa: E501

HEADER_TWO_ELEMENTS = 'ROS:11,1:313901,988.53;314843,988.57;315785,988.99;316727,991.56;317669,996.21;318611,1002.36;319553,1010.36;320495,1021.11;321437,1034.42;322379,1050.23;323321,1051.86;324263,1053.20;325206,1053.32;326146,1053.46;327088,1053.52;328030,1053.58;328972,1053.61;329914,1053.64;330856,1053.61;331798,1053.60;332740,1053.65;333682,1053.58;334624,1053.53;335566,1053.52;336508,1053.39;337453,1053.41;338395,1053.46;339337,1053.37;340279,1053.42;341221,1053.49;342163,1053.54'  # noqa: E501


ROS_SINGLE_ONE = 'ROS:SINGLE:25:5552,992.4500'
ROS_SINGLE_TWO = 'ROS:SINGLE:25:11071,994.4299'
ROS_SINGLE_THREE = 'ROS:SINGLE:25:16592,992.9600'
ROS_SINGLE_FOUR = 'ROS:SINGLE:25:22112,993.3699'
ROS_SINGLE_FIVE = 'ROS:SINGLE:25:27631,993.2600'

ROS_SINGLE_MSGS = (
    ROS_SINGLE_ONE,
    ROS_SINGLE_TWO,
    ROS_SINGLE_THREE,
    ROS_SINGLE_FOUR,
    ROS_SINGLE_FIVE,
)
TEST_VALUES = (992.4500, 994.4299, 992.9600, 993.3699, 993.26)
EXPECTED_VALUES = (992.4500, 993.43995, 993.27996666666, 993.30245, 993.29396)


@pytest.fixture
def packet_handler() -> SerialReaderPacketHandler:
    return SerialReaderPacketHandler()


def test_message_parser(packet_handler: SerialReaderPacketHandler) -> None:
    msg = packet_handler.message_parser(PACKET)

    assert msg == FloatData(
        team_number=11,
        profile_number=1,
        profile_half=1,
        time_data=[
            5.231683254241943,
            5.247383117675781,
            5.263083457946777,
            5.278783321380615,
            5.294483184814453,
            5.310183525085449,
            5.325883388519287,
            5.341583251953125,
            5.357283115386963,
            5.372983455657959,
            5.388683319091797,
            5.404383182525635,
            5.420100212097168,
            5.435766696929932,
            5.4514665603637695,
            5.467166900634766,
            5.4828667640686035,
            5.498566627502441,
            5.514266490936279,
            5.529966831207275,
            5.545666694641113,
            5.561366558074951,
            5.577066898345947,
            5.592766761779785,
            5.608466625213623,
            5.624216556549072,
            5.639916896820068,
            5.655616760253906,
            5.671316623687744,
            5.687016487121582,
            5.702716827392578,
        ],  # 501
        depth_data=[
            0.3828616142272949,
            0.38326960802078247,
            0.38755351305007935,
            0.41376692056655884,
            0.46119585633277893,
            0.5239244699478149,
            0.605522632598877,
            0.7151702046394348,
            0.8509292006492615,
            1.0121876001358032,
            1.0288132429122925,
            1.0424809455871582,
            1.043704867362976,
            1.0451328754425049,
            1.0457448959350586,
            1.0463569164276123,
            1.0466628074645996,
            1.0469688177108765,
            1.0466628074645996,
            1.0465608835220337,
            1.047070860862732,
            1.0463569164276123,
            1.0458468198776245,
            1.0457448959350586,
            1.0444189310073853,
            1.0446228981018066,
            1.0451328754425049,
            1.0442149639129639,
            1.044724941253662,
            1.0454388856887817,
            1.04594886302948,
        ],  # 501
    )

    with pytest.raises(ValueError, match='Packet expected 3 sections, found 2 sections'):
        packet_handler.message_parser(NOT_THREE_SECTIONS)

    with pytest.raises(ValueError, match='Packet header length of 3 expected found 2 instead'):
        packet_handler.message_parser(HEADER_TWO_ELEMENTS)


def test_handle_ros_single(packet_handler: SerialReaderPacketHandler) -> None:
    test_queue: Queue[float] = Queue(5)

    for ros_single, test_value, expected_value in zip(
        ROS_SINGLE_MSGS, TEST_VALUES, EXPECTED_VALUES, strict=True
    ):
        packet_handler.handle_ros_single(ros_single)
        test_queue.put(test_value)
        assert equal(packet_handler.surface_pressures, test_queue)
        assert pytest.approx(packet_handler.surface_pressure) == expected_value

    # Test no more get added
    packet_handler.handle_ros_single(ROS_SINGLE_FIVE)
    assert equal(packet_handler.surface_pressures, test_queue)
    assert pytest.approx(packet_handler.surface_pressure) == 993.29396


def equal(q1: Queue[T], q2: Queue[T]) -> bool:
    if isinstance(q1.queue, deque) and isinstance(q2.queue, deque):
        return q1.queue == q2.queue
    return False
