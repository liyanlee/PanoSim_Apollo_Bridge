import socket
import struct
import cv2
import numpy as np
from DataInterfacePython import *


def ModelStart(userData):
    # print(userData)
    userData["last"] = 0
    userData["width"] = int(userData["parameters"]["ResolutionWidth"])
    userData["height"] = int(userData["parameters"]["ResolutionHeight"])
    userData["bus"] = BusAccessor(userData["busId"], userData["parameters"]["CameraName"], "time@i,%d@[,r@b,g@b,b@b" % (userData["width"] * userData["height"]))
    userData["compression"] = userData["parameters"]["Compression"] == 'True'
    userData["sock"] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    userData["sock"].connect((userData["parameters"]["RemoteIP"], int(userData["parameters"]["Port"])))


def ModelOutput(userData):
    ts, _ = userData["bus"].readHeader()
    if ts > userData["last"]:
        userData["last"] = ts
        if userData["compression"]:
            img = np.frombuffer(userData["bus"].getBus()[8:], dtype=np.uint8).reshape((userData["height"], userData["width"], 3))
            bgr = cv2.cvtColor(img , cv2.COLOR_RGB2BGR)
            jpg_data = cv2.imencode('.jpg', bgr)[1].tobytes()
            header = struct.pack("<ii", ts, len(jpg_data))
            userData["sock"].send(header)
            userData["sock"].send(jpg_data)
        else:
            send_length = 8 + userData["width"] * userData["height"] * 3
            userData["sock"].send(userData["bus"].getBus()[0:send_length])


def ModelTerminate(userData):
    userData["sock"].close()
