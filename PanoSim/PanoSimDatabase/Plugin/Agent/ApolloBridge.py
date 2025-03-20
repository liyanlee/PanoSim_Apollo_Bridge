import socket
import struct
import math
import ctypes
import numpy as np

from DataInterfacePython import *


def ModelStart(userData):
    userData['send_route_complete'] = userData['parameters']['subRoute'] != 'True'

    bus_ego_format = 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d'
    userData['bus_ego'] = BusAccessor(userData['busId'], 'ego', bus_ego_format)

    bus_ego_extra_format = 'time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d'
    userData['bus_ego_extra'] = BusAccessor(userData['busId'], 'ego_extra', bus_ego_extra_format)

    bus_ego_animator_format = 'time@i,rot_spd_engine@d,trans_gear_position@d,engine_throttle_position@d,master_cyl_press@d,str_whl_ang@d,4@[,center_x@d,center_y@d,center_z@d,yaw@d,pitch@d,roll@d,rotation_spd@d,force_tire_x@d,force_tire_y@d,force_tire_z@d,gnd_x@d,gnd_y@d,gnd_z@d'
    userData['bus_ego_animator'] = BusAccessor(userData['busId'], 'ego_animator', bus_ego_animator_format)

    bus_traffic_light_format = 'time@i,64@[,id@i,direction@b,state@b,timer@i'
    userData['bus_traffic_light'] = BusAccessor(userData['busId'], 'traffic_light', bus_traffic_light_format)

    bus_traffic_format = 'time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f'
    userData['bus_traffic'] = DoubleBusReader(userData['busId'], 'traffic', bus_traffic_format)

    userData['send_obstacle'] = userData['parameters']['subObstacle'] == 'True'
    if userData['send_obstacle']:
        userData['sock_obstacles'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    userData['sock_localization'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData['sock_chassis'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData['sock_traffic_light'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    bus_xDriver_path_input_format = 'time@i,valid@b,500@[,x@d,y@d'
    userData['bus_xDriver_path_input'] = BusAccessor(userData['busId'], 'xDriver_path_input', bus_xDriver_path_input_format)
    bus_xDriver_speed_input_format = 'time@i,valid@b,speed@d,accel@d'
    userData['bus_xDriver_speed_input'] = BusAccessor(userData['busId'], 'xDriver_speed_input', bus_xDriver_speed_input_format)
    userData['sock_trajectory'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData['sock_trajectory'].setblocking(False)
    userData['sock_trajectory'].bind((userData['parameters']['LocalIP'], int(userData['parameters']['TrajectoryPort'])))

    userData['last_send_time'] = 0


def ModelOutput(userData):
    current_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData['bus_ego'].readHeader()
    recv_trajectory(userData, ego_x, ego_y, ego_yaw)

    if userData['time'] > 1000 and not userData['send_route_complete']:
        send_route(userData)
        userData['send_route_complete'] = True

    if current_time > userData['last_send_time']:
        userData['last_send_time'] = current_time
        apollo_ip = userData['parameters']['ApolloIP']

        _, _vx, _vy, _vz, _AVx, _AVy, _AVz, _Ax, _Ay, _Az, _, _, _ = userData['bus_ego_extra'].readHeader()
        format_localization = '<ddddddddddddddd'
        wheelbase = 2.578
        _x = ego_x - wheelbase * np.cos(ego_yaw)
        _y = ego_y - wheelbase * np.sin(ego_yaw)
        localization_data = struct.pack(format_localization, _x, _y, ego_z, ego_yaw, ego_pitch, ego_roll, _vx, _vy, _vz, _AVx, _AVy, _AVz, _Ax, _Ay, _Az)
        remote_port = userData['parameters']['LocalizationPort']
        userData['sock_localization'].sendto(localization_data, ((apollo_ip, int(remote_port))))

        _, width = userData['bus_traffic_light'].readHeader()
        if width > 0:
            remote_port = userData['parameters']['TrafficLightPort']
            real_size = 4 + 4 + (4 + 1 + 1 + 4) * width
            userData['sock_traffic_light'].sendto(userData['bus_traffic_light'].getBus()[0:real_size], ((apollo_ip, int(remote_port))))

        if userData['send_obstacle']:
            bus_reader = userData['bus_traffic'].getReader(userData['time'])
            _, width = bus_reader.readHeader()
            remote_port = userData['parameters']['ObstaclesPort']
            real_size = 3708
            userData['sock_obstacles'].sendto(bus_reader.getBus()[0:real_size], ((apollo_ip, int(remote_port))))

        values = userData['bus_ego_animator'].readHeader()
        trans_gear_position = values[2]
        if trans_gear_position > 1:
            trans_gear_position = 1
        engine_throttle_position = values[3] * 100
        master_cyl_press = values[4] / 15000000 * 100
        str_whl_ang = values[5] / (4 * np.pi) * 100
        format_chassis = '<ddddd'
        chassis_data = struct.pack(format_chassis, ego_speed, engine_throttle_position, master_cyl_press, str_whl_ang, trans_gear_position)
        remote_port = userData['parameters']['ChassisPort']
        userData['sock_chassis'].sendto(chassis_data, ((apollo_ip, int(remote_port))))


def ModelTerminate(userData):
    userData['sock_localization'].close()
    userData['sock_chassis'].close()
    userData['sock_trajectory'].close()
    if userData['send_obstacle']:
        userData['sock_obstacles'].close()


def filter_points(points):
    filter_list = []
    filter_count = 0
    find_first_point = False
    point_list = points.tolist()
    for pt in point_list:
        if find_first_point:
            last_x = filter_list[filter_count - 1][0]
            last_y = filter_list[filter_count - 1][1]
            distance = math.sqrt((pt[0] - last_x)**2 + (pt[1] - last_y)**2)
            if distance > 1:
                filter_list.append(pt)
                filter_count += 1
        else:
            if pt[0] > 0:
                filter_list.append(pt)
                filter_count = 1
                find_first_point = True
    return filter_list, filter_count


def send_route(userData):
    header_size = 8
    body_item_size = 16
    max_body_item_count = 1000
    offset = 0
    index = 0
    buffer = ctypes.create_string_buffer(header_size + max_body_item_count * 8)
    key_points = getKeyPoints()
    last_edge = -1
    for point in key_points:
        current_edge = getEdgeByLane(point[3])
        if current_edge >= 0 and last_edge != current_edge:
            offset = header_size + index * body_item_size
            struct.pack_into('<dd', buffer, offset, *(point[0], point[1]))
            index += 1

    if index > 1:
        struct.pack_into('<ii', buffer, 0, *(0, index))
        sock_route = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        real_size = header_size + index * body_item_size
        apollo_ip = userData['parameters']['ApolloIP']
        remote_port = userData['parameters']['RoutePort']
        sock_route.sendto(buffer[:real_size], ((apollo_ip, int(remote_port))))


def recv_trajectory(userData, ego_x, ego_y, ego_yaw):
    buffer = None
    header_size = 20
    buffer_max_size = header_size + 500 * (8 + 8)
    try:
        buffer, _ = userData['sock_trajectory'].recvfrom(buffer_max_size)
    except Exception as e:
        return
    if buffer:
        apollo_speed, apollo_accel, points_count = struct.unpack_from('<ddi', buffer)
        userData['bus_xDriver_speed_input'].writeHeader(*(userData['time'], 1, apollo_speed, apollo_accel))

        points = np.frombuffer(buffer[header_size:], dtype=np.float64).reshape((points_count, 2))

        move_matrix = np.array([[ego_x, ego_y]])
        points_move = points - move_matrix

        rotate_matrix = np.array([[np.cos(ego_yaw), np.sin(ego_yaw) * -1], [np.sin(ego_yaw), np.cos(ego_yaw)]])
        points_rotate = points_move @ rotate_matrix

        points_filter, points_count = filter_points(points_rotate)

        bus_path_input = userData['bus_xDriver_path_input']
        bus_path_input.writeHeader(*(userData['time'], 1, points_count))
        bus_path_input.getBus().seek(9)
        bus_path_input.getBus().write(np.array(points_filter, dtype='f8').tobytes())

