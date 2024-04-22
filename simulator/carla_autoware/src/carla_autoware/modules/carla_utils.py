# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.sr/bin/env python

import ctypes
import math
import struct
import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from transforms3d.euler import euler2quat


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    _DATATYPES = {}
    _DATATYPES[PointField.INT8] = ("b", 1)
    _DATATYPES[PointField.UINT8] = ("B", 1)
    _DATATYPES[PointField.INT16] = ("h", 2)
    _DATATYPES[PointField.UINT16] = ("H", 2)
    _DATATYPES[PointField.INT32] = ("i", 4)
    _DATATYPES[PointField.UINT32] = ("I", 4)
    _DATATYPES[PointField.FLOAT32] = ("f", 4)
    _DATATYPES[PointField.FLOAT64] = ("d", 8)

    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print("Skipping unknown PointField datatype [{}]" % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def create_cloud(header, fields, points):
    """Create a L{sensor_msgs.msg.PointCloud2} message with different datatype."""
    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))
    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=cloud_struct.size,
        row_step=cloud_struct.size * len(points),
        data=buff.raw,
    )


def carla_location_to_ros_point(carla_location):
    """Convert a carla location to a ROS point."""
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point


def carla_rotation_to_ros_quaternion(carla_rotation):
    """Convert a carla rotation to a ROS quaternion."""
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)
    quat = euler2quat(roll, pitch, yaw)
    ros_quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
    return ros_quaternion
