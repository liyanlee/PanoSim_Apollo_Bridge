import sys
import socket
import struct
import numpy as np

sys.path.append('/apollo/cyber/python')
sys.path.append('/apollo/bazel-bin')

from cyber_py3 import cyber, cyber_time
from modules.drivers.proto.pointcloud_pb2 import PointCloud, PointXYZIT


cyber.init()
print("PanoSim recv point cloud node start")

recv_point_cloud_node = cyber.Node("recv_point_cloud_node")
point_cloud_writer = recv_point_cloud_node.create_writer('/apollo/sensor/velodyne128/compensator/PointCloud2', PointCloud)


apollo_ip_address = '192.168.2.23'
port_recv_point_cloud = 14322

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
    server_sock.bind((apollo_ip_address, port_recv_point_cloud))
    server_sock.listen()
    while not cyber.is_shutdown():
        point_cloud_sequence_num = 1
        sock, addr = server_sock.accept()
        with sock:
            print(f"PanoSim connected: {addr}")
            while True:
                header = sock.recv(8, socket.MSG_WAITALL)
                if not header:
                    print(f"PanoSim disconnected: {addr}")
                    break
                ts, width = struct.unpack_from('<ii', header)
                if width <= 0:
                    break

                point_cloud_data = sock.recv(width * 16, socket.MSG_WAITALL)
                if not point_cloud_data:
                    break

                x_np_array = np.ndarray((width,), '<f', point_cloud_data, 0, (16,))
                y_np_array = np.ndarray((width,), '<f', point_cloud_data, 4, (16,))
                z_np_array = np.ndarray((width,), '<f', point_cloud_data, 8, (16,))
                intensity_np_array = np.ndarray((width, ), '<f', point_cloud_data, 12, (16,))

                point_cloud = PointCloud()
                point_cloud.header.sequence_num = point_cloud_sequence_num
                point_cloud.header.timestamp_sec = cyber_time.Time.now().to_sec()
                point_cloud.header.frame_id = 'velodyne128'
                point_cloud.frame_id = "velodyne128"
                point_cloud.is_dense = True

                # print(ts, width)

                for i in range(width):
                    pt = PointXYZIT()
                    pt.x = x_np_array[i]
                    pt.y = y_np_array[i]
                    pt.z = z_np_array[i]
                    pt.intensity = int(intensity_np_array[i])
                    point_cloud.point.append(pt)

                point_cloud.measurement_time = cyber_time.Time.now().to_nsec()
                point_cloud.width = width
                point_cloud.height = 1
                point_cloud_writer.write(point_cloud)
                point_cloud_sequence_num += 1


print("PanoSim recv point cloud node quit")

cyber.shutdown()
