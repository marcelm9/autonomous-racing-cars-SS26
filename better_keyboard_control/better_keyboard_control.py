#!/usr/bin/env python3
import sys
import threading
import rclpy
import select
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
s   : Emergency Stop (Sets speed to 0)

t   : TOGGLE RC-Style (Current: {mode})
      - RC-Style: Stops when key is released.
      - Standard: Maintains speed until changed.

r/v : increase/decrease max limits by 10%
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
        if msvcrt.kbhit():
            return msvcrt.getwch()
        return None
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None 
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
    
    # NEW: Toggle State
    rc_style = True 

    pub = node.create_publisher(AckermannDriveStamped, 'drive_in', 10)

    current_speed = 0.0
    current_steer = 0.0

    def timer_callback():
        nonlocal current_speed, current_steer
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = node.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = float(current_speed)
        drive_msg.drive.steering_angle = float(current_steer)
        pub.publish(drive_msg)

    timer = node.create_timer(0.1, timer_callback)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.daemon = True
    spinner.start()

    try:
        print(msg.format(mode="ON" if rc_style else "OFF"))
        while True:
            # Check for key with a short timeout
            key = getKey(settings, 0.1)
            
            if key in moveBindings:
                current_speed = moveBindings[key][0] * speed_limit
                current_steer = moveBindings[key][1] * steer_limit
            
            elif key in speedBindings:
                speed_limit *= speedBindings[key][0]
                steer_limit *= speedBindings[key][1]
                print(f"Limits: Speed={speed_limit:.2f}, Steer={steer_limit:.2f}")
            
            elif key == 't':
                rc_style = not rc_style
                print(f"RC-Style Toggled: {'ON' if rc_style else 'OFF'}")
                if rc_style:
                    current_speed = 0.0
                    current_steer = 0.0

            elif key == '\x03': # CTRL-C
                break
            
            else:
                # If no key is pressed and we are in RC-Style, stop the car
                if key is None and rc_style:
                    current_speed = 0.0
                    current_steer = 0.0

    except Exception as e:
        print(e)
    finally:
        current_speed = 0.0
        current_steer = 0.0
        # Give one last publish to stop the car
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        pub.publish(drive_msg)
        
        restoreTerminalSettings(settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()