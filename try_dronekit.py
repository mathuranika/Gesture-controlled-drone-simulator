from dronekit import connect
vehicle = connect('udp:127.0.0.1:14550')
print(vehicle)