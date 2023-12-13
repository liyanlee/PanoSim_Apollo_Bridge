import socket
from DataInterfacePython import *

def ModelStart(userData):
    userData["last"] = 0
    userData["frequency"] = int(userData["parameters"]["Frequency"])
    userData["max_count"] = int(userData["parameters"]["MaxCount"])
    userData["bus_lidar"] = BusAccessor(userData["busId"], userData["parameters"]["LidarName"], "time@i,%d@[,x@f,y@f,z@f,intensity@f" % (userData["max_count"]))
    userData["compression"] = userData["parameters"]["Compression"]
    userData["zip_lidar_data"] = BusAccessor(userData["busId"], "LidarZipData", "time@i,%d@[,x@f,y@f,z@f,intensity@f" % (userData["max_count"]))
    userData["sock"] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    userData["sock"].connect((userData["parameters"]["RemoteIP"], int(userData["parameters"]["Port"])))

def ModelOutput(userData):
    ts, width = userData["bus_lidar"].readHeader()
    if ts - userData["last"] >= 1000 / userData["frequency"]:
        userData["last"] = ts

        if userData["compression"] == 'True':
            bodyIndex = 0
            for i in range(width):
                x, y, z, intensity = userData["bus_lidar"].readBody(i)
                if intensity > 0:
                    userData["zip_lidar_data"].writeBody(bodyIndex, *(x, y, z, intensity))
                    bodyIndex += 1
            if bodyIndex > 0:
                userData["zip_lidar_data"].writeHeader(*(ts, bodyIndex))
                send_length = 8 + bodyIndex * 16
                userData["sock"].send(userData["zip_lidar_data"].getBus()[0:send_length])
                # print(userData['time'], bodyIndex)
        else:
            if width > 0:
                send_length = 8 + width * 16
                userData["sock"].send(userData["bus_lidar"].getBus()[0:send_length])

def ModelTerminate(userData):
    userData["sock"].close()
