#!/usr/bin/env python3
import sys
import threading
import rclpy
import select # Added for timeout handling
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
F1TENTH RC-Style Teleop
---------------------------
Hold keys to drive:
        w
   a    s    d
        x

w/x : Forward / Reverse
a/d : Steering Left / Right
s   : Emergency Stop

Release keys to coast to a stop.

r/v : increase/decrease max speed/steer by 10%
CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 0.0),   'x': (-1.0, 0.0),  
    'a': (0.0, 1.0),   'd': (0.0, -1.0),  
    's': (0.0, 0.0),
    'q': (1.0, 1.0),   'e': (1.0, -1.0),  
    'z': (-1.0, 1.0),  'c': (-1.0, -1.0), 
}

speedBindings = {
    'r': (1.1, 1.1),
    'v': (0.9, 0.9),
}

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # Windows doesn't easily support timeout-based keyboard polling in this way
        if msvcrt.kbhit():
            return msvcrt.getwch()
        return None
    
    tty.setraw(sys.stdin.fileno())
    # Use select to wait for input with a timeout
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None # No key pressed during timeout
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32': return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32' or old_settings is None: return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('better_keyboard_control')

    speed_limit = node.declare_parameter('max_speed', 1.5).value
    steer_limit = node.declare_parameter('max_steer', 0.34).value 

    pub = node.create_publisher(AckermannDriveStamped, 'drive_in', 10)

    # Use a Timer to ensure constant 10Hz publishing for safety_nodes
    current_speed = 0.0
    current_steer = 0.0

    def timer_callback():
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = node.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = current_speed
        drive_msg.drive.steering_angle = current_steer
        pub.publish(drive_msg)

    # Publish at 10Hz
    timer = node.create_timer(0.1, timer_callback)

    # Spinner in background
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.daemon = True
    spinner.start()

    try:
        print(msg)
        while True:
            # Check for key every 0.1 seconds
            key = getKey(settings, 0.1)
            
            if key in moveBindings:
                current_speed = moveBindings[key][0] * speed_limit
                current_steer = moveBindings[key][1] * steer_limit
            elif key in speedBindings:
                speed_limit *= speedBindings[key][0]
                steer_limit *= speedBindings[key][1]
                print(f"Limits: Speed={speed_limit:.2f}, Steer={steer_limit:.2f}")
            elif key == '\x03': # CTRL-C
                break
            else:
                # RC-STYLE: No key pressed = Reset to Zero
                current_speed = 0.0
                current_steer = 0.0

    except Exception as e:
        print(e)
    finally:
        current_speed = 0.0
        current_steer = 0.0
        timer_callback() # Final zero publish
        restoreTerminalSettings(settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()