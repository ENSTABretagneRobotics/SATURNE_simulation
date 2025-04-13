import roslibpy
import math
import time

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

# Global publisher variable to be used within callbacks
cmd_vel_topic = None

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

def main():
    print("Please change the IP address in the code and check your network configuration if needed!")

    # Create the connection to rosbridge (update with your robot's IP and port)
    ros = roslibpy.Ros(host='127.0.0.1', port=9090)
    
    # Create subscribers for sensor topics
    imu_topic = roslibpy.Topic(ros, '/imu/data', 'sensor_msgs/Imu')
    fix_topic = roslibpy.Topic(ros, '/fix', 'sensor_msgs/NavSatFix')
    button_topic = roslibpy.Topic(ros, '/pololu/button_0', 'std_msgs/Bool')
    
    imu_topic.subscribe(imu_callback)
    fix_topic.subscribe(fix_callback)
    button_topic.subscribe(button_callback)
    
    # Create a publisher for the motor command /cmd_vel topic
    global cmd_vel_topic
    cmd_vel_topic = roslibpy.Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')
    
    # Run the connection
    ros.run()
    
    print("Listening for IMU, GPS, and button events. Press CTRL+C to exit.")
    try:
        while True:
            start_motors()
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
        cmd_vel_topic.unadvertise()
        ros.terminate()

if __name__ == '__main__':
    main()
