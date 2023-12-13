import sys
import socket
import struct

sys.path.append('/apollo/cyber/python')
sys.path.append('/apollo/bazel-bin')

from cyber_py3 import cyber, cyber_time
from modules.drivers.proto.sensor_image_pb2 import CompressedImage


cyber.init()
print("PanoSim recv jpeg node start")

recv_jpeg_node = cyber.Node("recv_jpeg_node")
jpeg_writer = recv_jpeg_node.create_writer('/apollo/sensor/camera/front_6mm/image/compressed', CompressedImage)

apollo_ip_address = '192.168.2.23'
port_recv_jpeg = 14321

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
    server_sock.bind((apollo_ip_address, port_recv_jpeg))
    server_sock.listen()
    while not cyber.is_shutdown():
        jpeg_sequence_num = 1
        sock, addr = server_sock.accept()
        with sock:
            print(f"PanoSim connected: {addr}")
            while True:
                header = sock.recv(8)
                if not header:
                    break
                ts, width = struct.unpack_from('<ii', header)
                if width <= 0:
                    break

                jpeg_data = b""
                recv_length = 0
                while recv_length < width:
                    buffer = sock.recv(width - recv_length)
                    if buffer:
                        jpeg_data += buffer
                        recv_length = len(jpeg_data)
                if not jpeg_data:
                    break

                jpeg_msg = CompressedImage()
                jpeg_msg.header.sequence_num = jpeg_sequence_num
                jpeg_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                jpeg_msg.header.frame_id = 'front_6mm'
                jpeg_msg.measurement_time = cyber_time.Time.now().to_sec()
                jpeg_msg.format = 'jpeg'
                # jpeg_msg.format = 'mjpeg'
                jpeg_msg.data = bytes(jpeg_data)
                jpeg_writer.write(jpeg_msg)
                jpeg_sequence_num += 1

print("PanoSim recv jpeg node quit")

cyber.shutdown()
