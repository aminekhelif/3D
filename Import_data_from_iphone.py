import socket
import json
import time

# Use the local network IP address of the computer here
ip_address = '172.20.10.144'  # Replace with your computer's local network IP
port = 65000

# Create a socket for IPv4 (AF_INET) using UDP (SOCK_DGRAM)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Set the socket to broadcast and allow reusing addresses
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind the socket to the IP address and port
sock.bind((ip_address, port))

print("Listening for data on {}:{}".format(ip_address, port))

# Set a timeout so recvfrom won't block indefinitely
sock.settimeout(5)

# Listen for incoming datagrams
time_tick = time.time()
while True and time.time() - time_tick < 30:
    try:
        print("Waiting for data...")
        data, addr = sock.recvfrom(4096)  # buffer size is 4096 bytes
        print("Received message from", addr)
        try:
            # Decode the JSON data
            json_data = json.loads(data.decode())
            print("Received data:", json_data)
        except json.JSONDecodeError as e:
            print("Could not decode JSON data:", e)
    except socket.timeout:
        print("No data received. Timeout.")
    except Exception as e:
        print("An error occurred:", e)
