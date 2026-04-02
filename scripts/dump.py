import json
from dataclasses import asdict, dataclass
from enum import IntEnum
from typing import Sequence, TypeAlias


class Reader:
    def __init__(self, data: bytes) -> None:
        self._data = data

    def __bool__(self) -> bool:
        return len(self._data) != 0

    def next_int(self, size: int, signed: bool) -> int:
        value = int.from_bytes(self._data[:size], byteorder="little", signed=signed)
        self._data = self._data[size:]
        return value

    def next_u8(self) -> int:
        return self.next_int(1, False)

    def next_u16(self) -> int:
        return self.next_int(2, False)

    def next_u32(self) -> int:
        return self.next_int(4, False)

    def next_i16(self) -> int:
        return self.next_int(2, True)


@dataclass
class Point2d:
    x: int
    y: int

    @classmethod
    def parse_array(cls, reader: Reader, count: int) -> "Sequence[Point2d]":
        values = []
        for _ in range(count):
            x = reader.next_u16()
            y = reader.next_u16()
            values.append(cls(x, y))
        return values


@dataclass
class PathPoint:
    ix: int
    iy: int

    @classmethod
    def parse_array(cls, reader: Reader, count: int) -> "Sequence[PathPoint]":
        values = []
        for _ in range(count):
            x = reader.next_u8()
            y = reader.next_u8()
            values.append(cls(x, y))
        return values


@dataclass
class Move:
    type: str
    t: int
    delta_x: int
    delta_y: int
    delta_theta: int

    @classmethod
    def parse(cls, reader: Reader) -> "Move":
        t = reader.next_u32()
        delta_x = reader.next_i16()
        delta_y = reader.next_i16()
        delta_theta = reader.next_i16()
        return cls("move", t, delta_x, delta_y, delta_theta)


@dataclass
class Scan:
    type: str
    t: int
    border_points: Sequence[Point2d]
    obstacle_points: Sequence[Point2d]

    @classmethod
    def parse(cls, reader: Reader) -> "Scan":
        t = reader.next_u32()
        border_point_count = reader.next_u16()
        obstacle_point_count = reader.next_u16()
        border_points = Point2d.parse_array(reader, border_point_count)
        obstacle_points = Point2d.parse_array(reader, obstacle_point_count)
        return cls("scan", t, border_points, obstacle_points)


@dataclass
class EstimatedPose:
    type: str
    t: int
    x: int
    y: int
    theta: int

    @classmethod
    def parse(cls, reader: Reader) -> "EstimatedPose":
        t = reader.next_u32()
        x = reader.next_u16()
        y = reader.next_u16()
        theta = reader.next_i16()
        return cls("estimated_pose", t, x, y, theta)


@dataclass
class CurrentPose:
    type: str
    t: int
    x: int
    y: int
    theta: int

    @classmethod
    def parse(cls, reader: Reader) -> "CurrentPose":
        t = reader.next_u32()
        x = reader.next_u16()
        y = reader.next_u16()
        theta = reader.next_i16()
        return cls("current_pose", t, x, y, theta)


@dataclass
class Path:
    type: str
    t: int
    points: Sequence[PathPoint]

    @classmethod
    def parse(cls, reader: Reader) -> "Path":
        t = reader.next_u32()
        point_count = reader.next_u16()
        points = PathPoint.parse_array(reader, point_count)
        return cls("path", t, points)


@dataclass
class Motor:
    type: str
    t: int
    speed_a: int
    speed_b: int
    speed_c: int

    @classmethod
    def parse(cls, reader: Reader) -> "Motor":
        t = reader.next_u32()
        speed_a = reader.next_i16()
        speed_b = reader.next_i16()
        speed_c = reader.next_i16()
        return cls("motor", t, speed_a, speed_b, speed_c)


Frame: TypeAlias = Move | Scan | EstimatedPose | CurrentPose | Path | Motor


class FrameType(IntEnum):
    HEARTBEAT = 1
    MOVE = 2
    SCAN = 3
    ESTIMATED_POSE = 4
    CURRENT_POSE = 5
    PATH = 6
    MOTOR = 7


def parse_frames(reader: Reader) -> Sequence[Frame]:
    frames = []

    while reader:
        frame_type = reader.next_u8()

        if frame_type == FrameType.MOVE:
            frame = Move.parse(reader)
        elif frame_type == FrameType.SCAN:
            frame = Scan.parse(reader)
        elif frame_type == FrameType.ESTIMATED_POSE:
            frame = EstimatedPose.parse(reader)
        elif frame_type == FrameType.CURRENT_POSE:
            frame = CurrentPose.parse(reader)
        elif frame_type == FrameType.PATH:
            frame = Path.parse(reader)
        elif frame_type == FrameType.MOTOR:
            frame = Motor.parse(reader)
        else:
            continue

        frames.append(frame)

    return frames


@dataclass
class Capture:
    version: int
    frames: Sequence[Frame]

    @classmethod
    def parse(cls, reader: Reader) -> "Capture":
        version = reader.next_u32()
        frames = parse_frames(reader)
        return cls(version, frames)


with open("test/capture", "rb") as f:
    data = f.read()

reader = Reader(data)
capture = Capture.parse(reader)
print(json.dumps(asdict(capture), indent=2))
