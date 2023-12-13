import sys
import ctypes
import struct
import socket

sys.path.append('/apollo/cyber/python')
sys.path.append('/apollo/bazel-bin')

from cyber_py3 import cyber
from modules.planning.proto.planning_pb2 import ADCTrajectory


cyber.init()
print("send trajectory node start")

node_send_trajectory = cyber.Node("send_trajectory_node")

panosim_ip_address = '192.168.2.33'
panosim_trajectory_port = 12365
sock_send_trajectory = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_send_trajectory.setblocking(True)

header_size = 8 + 8 + 4
body_item_size = 8 * 2
max_body_count = 500
body_total_size = body_item_size * max_body_count

buffer = ctypes.create_string_buffer(header_size + body_total_size)

def trajectory_callback(data):
    points = data.trajectory_point
    width = 0
    speed = 0.0
    accel = 0.0
    real_size = header_size
    for pt in points:
        if pt.path_point.s > 2 and speed < 0.01:
            speed = pt.v
            accel = pt.a
        offset = header_size + width * body_item_size
        struct.pack_into('<dd', buffer, offset, *(pt.path_point.x, pt.path_point.y))
        real_size += body_item_size
        width += 1
        if width >= max_body_count:
            break
    if width > 0:
        struct.pack_into('<ddi', buffer, 0, *(speed, accel, width))
        sock_send_trajectory.sendto(buffer[:real_size], ((panosim_ip_address, panosim_trajectory_port)))

node_send_trajectory.create_reader('/apollo/planning', ADCTrajectory, trajectory_callback)
node_send_trajectory.spin()

print("send trajectory node quit")
cyber.shutdown()
