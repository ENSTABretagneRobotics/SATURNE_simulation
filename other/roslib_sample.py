import roslibpy
import math
import time
import math
import time
import base64
import struct

# Global variables to be used within callbacks
cmd_vel_topic = None
obstacle = False

def quaternion_to_yaw(q):
    x = q.get('x', 0.0)
    y = q.get('y', 0.0)
    z = q.get('z', 0.0)
    w = q.get('w', 0.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 2.0 * (w * w + x * x) - 1.0
    return math.atan2(siny_cosp, cosy_cosp)

def imu_callback(message):
    orientation = message.get('orientation', {})
    yaw = -quaternion_to_yaw(orientation) # Approximate conversion...
    print(f"[IMU] Yaw: {math.degrees(yaw):.2f} deg")

def fix_callback(message):
    latitude = message.get('latitude', None)
    longitude = message.get('longitude', None)
    if latitude is not None and longitude is not None:
        print(f"[GPS] Latitude: {latitude}, Longitude: {longitude}")
    else:
        print("[GPS] Data not available.")

def start_motors():
    global cmd_vel_topic
    msg = roslibpy.Message({
        'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.2}
    })
    cmd_vel_topic.publish(msg)
    print("[CMD_VEL] Published start command.")

def stop_motors():
    global cmd_vel_topic
    msg = roslibpy.Message({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })
    cmd_vel_topic.publish(msg)
    print("[CMD_VEL] Published stop command.")

def button_callback(message):
    pressed = message.get('data', False)
    if pressed: print("[BUTTON] Pressed")
    else: print("[BUTTON] Released")

def decode_pointcloud2(msg):
    """
    Decodes a sensor_msgs/PointCloud2 message into a list of (x, y, z) points.
    Assumes that x, y, and z are stored as 32-bit floats.
    """
    points = []
    
    width = msg.get('width', 0)
    height = msg.get('height', 0)
    point_step = msg.get('point_step', 0)
    fields = msg.get('fields', [])
    is_bigendian = msg.get('is_bigendian', False)
    data = msg.get('data', "")
    if data == "":
        return points
    # Decode the base64 encoded data to a binary blob
    data = base64.b64decode(data)

    # Find offsets for x, y and z
    x_offset = y_offset = z_offset = None
    for f in fields:
        if f.get('name') == 'x':
            x_offset = f.get('offset')
        elif f.get('name') == 'y':
            y_offset = f.get('offset')
        elif f.get('name') == 'z':
            z_offset = f.get('offset')
    if x_offset is None or y_offset is None or z_offset is None:
        print("[Velodyne] Missing x, y, or z field in point cloud data")
        return points

    fmt_char = '>' if is_bigendian else '<'
    for i in range(width * height):
        offset = i * point_step
        try:
            x = struct.unpack_from(fmt_char + 'f', data, offset + x_offset)[0]
            y = struct.unpack_from(fmt_char + 'f', data, offset + y_offset)[0]
            z = struct.unpack_from(fmt_char + 'f', data, offset + z_offset)[0]
        except struct.error:
            continue
        points.append((x, y, z))
    return points

def velodyne_callback(message):
    """
    Callback for the /velodyne_points topic.
    
    Processes the point cloud to check for obstacles in the horizontal plane.
    An obstacle is considered if:
      - The point is in front of the sensor (x > 0),
      - Its height is near 0 (|z| is below a defined threshold),
      - And its horizontal distance sqrt(x^2+y^2) is less than 1 m.
    """
    global obstacle

    points = decode_pointcloud2(message)
    if not points:
        print("[Velodyne] No points decoded.")
        return

    z_threshold = 0.2             # Consider points with |z| < 0.2 m as in the horizontal plane
    obstacle_distance_threshold = 1.0  # Only report obstacles closer than 1 meter
    
    obstacle_detected = False

    for (x, y, z) in points:
        if x <= 0:
            continue  # Only consider points in front of the sensor
        if abs(z) > z_threshold:
            continue  # Point is not in the horizontal plane
        horizontal_distance = math.sqrt(x * x + y * y)
        if horizontal_distance < obstacle_distance_threshold:
            print(f"[Velodyne] Obstacle detected at horizontal distance: {horizontal_distance:.2f} m, z: {z:.2f} m")
            obstacle_detected = True
            break  # Report the first detected obstacle and exit the loop

    if obstacle_detected: obstacle = True
    else: obstacle = False

def main():
    global obstacle

    print("Please change the IP address in the code and check your network configuration if needed!")

    # Create the connection to rosbridge (update with your robot's IP and port)
    ros = roslibpy.Ros(host='192.168.1.200', port=9090)
    
    # Create subscribers for sensor topics
    imu_topic = roslibpy.Topic(ros, '/imu/data', 'sensor_msgs/Imu')
    fix_topic = roslibpy.Topic(ros, '/fix', 'sensor_msgs/NavSatFix')
    button_topic = roslibpy.Topic(ros, '/pololu/button_0', 'std_msgs/Bool')
    velodyne_topic = roslibpy.Topic(ros, '/velodyne_points', 'sensor_msgs/PointCloud2')
   
    imu_topic.subscribe(imu_callback)
    fix_topic.subscribe(fix_callback)
    button_topic.subscribe(button_callback)
    velodyne_topic.subscribe(velodyne_callback)
    
    # Create a publisher for the motor command /cmd_vel topic
    global cmd_vel_topic
    cmd_vel_topic = roslibpy.Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')
    
    # Run the connection
    ros.run()
    
    print("Moving and listening for IMU, GPS, button, and Velodyne events. Press CTRL+C to exit.")
    try:
        while True:
            if not obstacle: start_motors()
            # Idle loop keeps the script running.
            time.sleep(1)
    except KeyboardInterrupt:
        print("CTRL+C pressed. Shutting down...")
    finally:
        stop_motors()
        time.sleep(1)
        # Clean up: unsubscribe and end the connection.
        imu_topic.unsubscribe()
        fix_topic.unsubscribe()
        button_topic.unsubscribe()
        velodyne_topic.unsubscribe()
        cmd_vel_topic.unadvertise()
        ros.terminate()

if __name__ == '__main__':
    main()
