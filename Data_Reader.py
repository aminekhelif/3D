import socket
import json
import numpy as np
# def read_sensor_data(ip_address='172.20.10.144', port=int(65000)):
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
#     try:
#         # Bind the socket to the IP address and port
#         sock.bind((ip_address, port))
        
#         # Set a timeout in case the data is not received
#         sock.settimeout(5.0)
        
#         # Wait for a single message
#         data, addr = sock.recvfrom(4096)  # buffer size is 4096 bytes
        
#         # Load the JSON data
#         json_data = json.loads(data.decode())
        
#         # Extract the relevant sensor data
#         sensor_values = [
#             json_data['gyroTime'],  # Use 'gyroTime' as a common timestamp
#             json_data['gyroX'],
#             json_data['gyroY'],
#             json_data['gyroZ'],#0,0,0
#             json_data['accelX'],
#             json_data['accelY'],0
#             # json_data['accelZ']
#         ]
        
#         # Convert to a numpy array
#         sensor_array = np.array(sensor_values)
        
#     except socket.timeout:
#         print("No data received within the timeout period")
#         sensor_array = np.array([])
#     finally:
#         # Close the socket
#         sock.close()
    
#     return sensor_array


def read_sensor_data(i):
    T=0.1
    f =1/T
    time_vector = np.arange(0, 10, T)
    gyroX = np.zeros(len(time_vector))
    gyroX[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    # gyroY = np.array([0, 0, 0, 0, 0, 0, 0.5*np.pi, 0.5*np.pi, 0.5*np.pi, 0.5*np.pi])
    gyroY = np.zeros(len(time_vector))
    gyroY[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    gyroZ = np.zeros(len(time_vector))
    gyroZ[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    # gyroY = np.zeros(len(time_vector))
    # gyroZ = np.zeros(len(time_vector))
    accelX =    np.zeros(len(time_vector)) 
    accelY =    np.zeros(len(time_vector))
    accelZ =    np.zeros(len(time_vector))

    Sensor_data = np.vstack((time_vector, gyroX, gyroY, gyroZ, accelX, accelY, accelZ)).T
    return Sensor_data[i]

    