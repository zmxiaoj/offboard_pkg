#!/usr/bin/env python3
import rospy
import signal
from std_msgs.msg import String, Bool

# Command mapping dictionary
TRAJECTORY_COMMANDS = {
    'h': 'hover',
    'c': 'circle',
    'ch': 'circle_head',
    'r': 'rectangle',
    'rh': 'rectangle_head',
    'e': 'eight',
    'eh': 'eight_head',
    'l': 'land',
    'q': 'quit'
}

def signal_handler(signum, frame):
    rospy.loginfo("\nCaught Ctrl+C, performing clean shutdown...")
    rospy.signal_shutdown('User requested shutdown')
    exit(0)

def print_help():
    help_msg = """
Available commands:
    h  or hover           - Hover in place
    c  or circle         - Circle trajectory without heading control
    ch or circle_head    - Circle trajectory with heading control
    r  or rectangle     - Rectangle trajectory without heading control
    rh or rectangle_head - Rectangle trajectory with heading control
    e  or eight         - Figure-8 trajectory without heading control
    eh or eight_head    - Figure-8 trajectory with heading control
    l  or land          - Land the drone
    q  or quit          - Exit the program
    help                - Show this help message
    """
    print(help_msg)

def send_trajectory_cmd():
    rospy.init_node('trajectory_cmd_sender', anonymous=True)
    traj_pub = rospy.Publisher('/trajectory_cmd', String, queue_size=10)
    land_pub = rospy.Publisher('/land_cmd', Bool, queue_size=10)
    rate = rospy.Rate(1)

    signal.signal(signal.SIGINT, signal_handler)
    print_help()
    
    while not rospy.is_shutdown():
        try:
            cmd = input("\nEnter command (or 'help' for options): ").strip().lower()
            if not cmd:
                rospy.logwarn("Empty input, please try again")
                continue
            
            # Show help message
            if cmd == 'help':
                print_help()
                continue
                
            # Handle full command names
            if cmd in TRAJECTORY_COMMANDS.values():
                actual_cmd = cmd
            # Handle command shortcuts
            elif cmd in TRAJECTORY_COMMANDS:
                actual_cmd = TRAJECTORY_COMMANDS[cmd]
            else:
                rospy.logwarn("Invalid command: %s (type 'help' for options)", cmd)
                continue

            if actual_cmd in ['quit', 'q']:
                rospy.loginfo("Exiting command sender...")
                break
            elif actual_cmd == 'land':
                msg = Bool()
                msg.data = True
                land_pub.publish(msg)
                rospy.loginfo("Land command sent")
                break
            else:
                msg = String()
                msg.data = actual_cmd
                traj_pub.publish(msg)
                rospy.loginfo("Trajectory command sent: %s", actual_cmd)
                
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

if __name__ == '__main__':
    try:
        send_trajectory_cmd()
    except rospy.ROSInterruptException:
        pass