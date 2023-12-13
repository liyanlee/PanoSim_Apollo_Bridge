import sys
import math
import struct
import socket
import select
from scipy.spatial.transform import Rotation

sys.path.append('/apollo/cyber/python')
sys.path.append('/apollo/bazel-bin')

from cyber_py3 import cyber, cyber_time
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLight, TrafficLightDetection
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint


cyber.init()
print("panosim bridge node start")

panosim_bridge = cyber.Node("panosim_bridge_node")

localization_writer = panosim_bridge.create_writer('/apollo/localization/pose', LocalizationEstimate)
chassis_writer = panosim_bridge.create_writer('/apollo/canbus/chassis', Chassis)
obstacle_writer = panosim_bridge.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
traffic_light_writer = panosim_bridge.create_writer('/apollo/perception/traffic_light', TrafficLightDetection)
tf_writer = panosim_bridge.create_writer( "/tf", TransformStampeds)
routing_writer = panosim_bridge.create_writer('/apollo/routing_request', RoutingRequest)

local_ip_address = '192.168.2.23'
port_localization = 12361
format_localization = '<ddddddddddddddd'
size_localization = 10 * 13

port_chassis = 12362
format_chassis = '<ddddd'
size_chassis = 8 * 13

port_obstacles = 12363
size_obstacles = 3708

format_routing_header = '<ii'
format_routing_body = '<dd'
size_routing_body_item = 16
port_routing = 12364
size_routing = 8 + 1000 * 16
route_points = []

port_traffic_light = 12366
format_traffic_light = '<ibbi'
size_traffic_light = 64 * (4 + 1 + 1 + 4)


sock_localization = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_localization.bind((local_ip_address, port_localization))

sock_chassis = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_chassis.bind((local_ip_address, port_chassis))

sock_obstacles = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_obstacles.bind((local_ip_address, port_obstacles))

sock_traffic_light = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_traffic_light.bind((local_ip_address, port_traffic_light))

sock_routing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_routing.bind((local_ip_address, port_routing))

rfds = [sock_localization, sock_chassis, sock_obstacles, sock_traffic_light, sock_routing]
select_timeout_second = 0.1


def on_recv_routing(sock):
    try:
        route_points.clear()
        routing_data, _ = sock.recvfrom(size_routing)
    except Exception as e:
        print(e)
    if routing_data:
        _, points_count = struct.unpack_from(format_routing_header, routing_data)
        if points_count > 0:
            for i in range(points_count):
                x, y = struct.unpack_from(format_routing_body, routing_data, 8 + i * size_routing_body_item)
                route_points.append([x, y])


def on_recv_localization(sock):
    try:
        localization_data, _ = sock.recvfrom(size_localization)
    except Exception as e:
        print(e)
    if localization_data:
        ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, vx, vy, vz, avx, avy, avz, ax, ay, az = struct.unpack_from(format_localization, localization_data)
        localization_msg = LocalizationEstimate()
        localization_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        localization_msg.header.frame_id = 'novatel'

        localization_msg.pose.position.x = ego_x
        localization_msg.pose.position.y = ego_y
        localization_msg.pose.position.z = ego_z
        localization_msg.pose.linear_velocity.x = vx
        localization_msg.pose.linear_velocity.y = vy
        localization_msg.pose.linear_velocity.z = vz
        localization_msg.pose.angular_velocity_vrf.x = avx
        localization_msg.pose.angular_velocity_vrf.y = avy
        localization_msg.pose.angular_velocity_vrf.z = avz
        localization_msg.pose.linear_acceleration_vrf.x = ax
        localization_msg.pose.linear_acceleration_vrf.y = ay
        localization_msg.pose.linear_acceleration_vrf.z = az

        rotation = Rotation.from_euler('ZXY', [ego_yaw - (math.pi / 2), ego_pitch, ego_roll], degrees=False)
        quat = rotation.as_quat()

        localization_msg.pose.orientation.qx = quat[0]
        localization_msg.pose.orientation.qy = quat[1]
        localization_msg.pose.orientation.qz = quat[2]
        localization_msg.pose.orientation.qw = quat[3]
        localization_msg.pose.heading = ego_yaw
        localization_writer.write(localization_msg)

        tf_msg = TransformStampeds()
        tf_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        tf_msg.transforms.add()
        tf_msg.transforms[0].header.frame_id = 'world'
        tf_msg.transforms[0].child_frame_id = 'localization'
        tf_msg.transforms[0].transform.translation.x = ego_x
        tf_msg.transforms[0].transform.translation.y = ego_y
        tf_msg.transforms[0].transform.translation.z = ego_z

        tf_msg.transforms[0].transform.rotation.qx = quat[0]
        tf_msg.transforms[0].transform.rotation.qy = quat[1]
        tf_msg.transforms[0].transform.rotation.qz = quat[2]
        tf_msg.transforms[0].transform.rotation.qw = quat[3]

        tf_writer.write(tf_msg)

        route_points_count = len(route_points)
        if route_points_count > 1:
            routing_msg = RoutingRequest()
            routing_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            for i in range(route_points_count):
                way_point = LaneWaypoint()
                way_point.pose.x = route_points[i][0]
                way_point.pose.y = route_points[i][1]
                routing_msg.waypoint.append(way_point)
            routing_writer.write(routing_msg)
            route_points.clear()


def on_recv_chassis(sock):
    try:
        chassis_data, _ = sock.recvfrom(size_chassis)
    except Exception as e:
        print(e)
    if chassis_data:
        v1, v2, v3, v4, v5 = struct.unpack_from(format_chassis, chassis_data)
        chassis_msg = Chassis()
        chassis_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        chassis_msg.engine_started = True
        chassis_msg.speed_mps = v1# position['vx']
        chassis_msg.throttle_percentage = v2# position['throttle'] * 100
        chassis_msg.brake_percentage = v3# position['brake'] / 5500000 * 100
        chassis_msg.steering_percentage = v4# position['steerangle'] * 10
        chassis_msg.driving_mode = 1
        # chassis_msg.gear_location = int(v5)
        chassis_writer.write(chassis_msg)


def on_recv_obstacles(sock):
    try:
        obstacles_data, _ = sock.recvfrom(size_obstacles)
    except Exception as e:
        print(e)
    if obstacles_data:
        format_obstacles_header = '<ii'
        obstacles_header_size = 2 * 4
        ts, width = struct.unpack_from(format_obstacles_header, obstacles_data)
        if width > 0:
            obstacles = PerceptionObstacles()
            obstacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
            format_obstacle_body = '<ibifffffff'
            obstacle_body_size = 4 + 1 + 4 + 4 * 7
            vehicles = []
            index = 0
            for index in range(width):
                v_id, _, _, v_x, v_y, v_z, v_yaw, _, _, v_speed = struct.unpack_from(format_obstacle_body, obstacles_data, obstacles_header_size + index * obstacle_body_size)
                v_yaw = math.radians(90 - v_yaw)
                obstacle = PerceptionObstacle()
                obstacle.id = v_id
                obstacle.position.x = v_x
                obstacle.position.y = v_y
                obstacle.position.z = v_z
                obstacle.theta = v_yaw
                obstacle.velocity.x = v_speed * math.cos(v_yaw)
                obstacle.velocity.y = v_speed * math.sin(v_yaw)
                obstacle.velocity.z = 0
                obstacle.length = 5
                obstacle.width = 2
                obstacle.height = 2
                vehicles.append(obstacle)
            obstacles.perception_obstacle.extend(vehicles)
            obstacle_writer.write(obstacles)
        else:
            obstacles = PerceptionObstacles()
            obstacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
            vehicles = []
            obstacles.perception_obstacle.extend(vehicles)
            obstacle_writer.write(obstacles)


def on_recv_traffic_light(sock):
    try:
        traffic_light_data, addr = sock.recvfrom(size_traffic_light)
    except Exception as e:
        print(e)
    if traffic_light_data:
        format_traffic_light_header = '<ii'
        traffic_light_header_size = 4 + 4
        ts, width = struct.unpack_from(format_traffic_light_header, traffic_light_data)
        if width > 0:
            traffic_light_detection = TrafficLightDetection()
            traffic_light_detection.header.timestamp_sec = cyber_time.Time.now().to_sec()
            traffic_light = []

            format_traffic_light_body = '<ibbi'
            traffic_light_body_size = 4 + 1 + 1 + 4

            index = 0
            for index in range(width):
                id, _, state, _ = struct.unpack_from(format_traffic_light_body, traffic_light_data, traffic_light_header_size + index * traffic_light_body_size)
                tf = TrafficLight()
                tf.color = state + 1
                tf.id = str(id)
                traffic_light.append(tf)
            traffic_light_detection.traffic_light.extend(traffic_light)
            traffic_light_writer.write(traffic_light_detection)


while not cyber.is_shutdown():
    readable, writeable, exceptional = select.select(rfds, [], rfds, select_timeout_second)
    for sock in readable:
        if sock == sock_localization:
            on_recv_localization(sock)
        elif sock == sock_chassis:
            on_recv_chassis(sock)
        elif sock == sock_obstacles:
            on_recv_obstacles(sock)
        elif sock == sock_traffic_light:
            on_recv_traffic_light(sock)
        elif sock == sock_routing:
            on_recv_routing(sock)


print("panosim bridge node quit")
cyber.shutdown()
