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
    time_vector = np.arange(0, 11, T)
    # gyroX = np.zeros(len(time_vector))
    # gyroX[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    # # gyroY = np.array([0, 0, 0, 0, 0, 0, 0.5*np.pi, 0.5*np.pi, 0.5*np.pi, 0.5*np.pi])
    # gyroY = np.zeros(len(time_vector))
    # gyroY[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    # gyroZ = np.zeros(len(time_vector))
    # gyroZ[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)
    # # gyroY = np.zeros(len(time_vector))
    # # gyroZ = np.zeros(len(time_vector))
    # accelX =    np.zeros(len(time_vector)) 
    # accelY =    np.zeros(len(time_vector))
    # accelZ =    np.ones(len(time_vector))*-9.81
    
    
    # gyroX = np.zeros(len(time_vector))
    # gyroX[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)

    # gyroY = np.zeros(len(time_vector))
    # gyroY[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)

    # gyroZ = np.zeros(len(time_vector))
    # gyroZ[int(3*len(time_vector)/f):int(7*len(time_vector)/f)] = np.radians(90)

    # accelX = np.zeros(len(time_vector)) 
    # accelY = np.zeros(len(time_vector))
    # accelZ = np.ones(len(time_vector)) * -9.81
    
    
    #  ------------------  Spiral Motion  ------------------
    
    radius = 3  # Radius of the spiral
    a = 0.1  # Spiral pitch
    total_points = len(time_vector)
    
    
    x_extended = radius * np.cos(time_vector)
    y_extended = radius * np.sin(time_vector)
    z_extended = a * time_vector

    # Calculate velocities in x, y, and z
    vx = np.gradient(x_extended, time_vector)
    vy = np.gradient(y_extended, time_vector)
    vz = np.gradient(z_extended, time_vector)
    # vx = np.diff(x_extended)/T
    # vy = np.diff(y_extended)/T
    # vz = np.diff(z_extended)/T
    # Calculate accelerations in x, y, and z
    accelX_extended = np.gradient(vx, time_vector)
    accelY_extended = np.gradient(vy, time_vector)
    accelZ_extended = np.gradient(vz, time_vector) - 9.81
    # accelX_extended = np.diff(vx)/T
    # accelX_extended.resize(total_points)
    # accelY_extended = np.diff(vy)/T
    # accelY_extended.resize(total_points)
    # accelZ_extended = np.diff(vz)/T
    # accelZ_extended.resize(total_points)
    # accelZ_extended = accelZ_extended - 9.81
    
    # Calculate angular rates (gyroscope data)
    # For simplicity, assuming rotation around z-axis only
    theta = np.arctan2(y_extended, x_extended)
    gyroZ_extended = np.gradient(theta, time_vector)
    gyroX_extended = np.zeros(total_points)  # No rotation around X-axis
    gyroY_extended = np.zeros(total_points)  # No rotation around Y-axis

    # Combine IMU data into a single array
    Sensor_data = np.vstack((time_vector, gyroX_extended, gyroY_extended, gyroZ_extended, accelX_extended, accelY_extended, accelZ_extended)).T


    # Sensor_data = np.vstack((time_vector, gyroX, gyroY, gyroZ, accelX, accelY, accelZ)).T
    return Sensor_data[i]

    