#!/usr/bin/env python3
import rospy
import signal
from std_msgs.msg import String, Bool

def signal_handler(signum, frame):
    rospy.loginfo("\nCaught Ctrl+C, performing clean shutdown...")
    rospy.signal_shutdown('User requested shutdown')
    exit(0)

def send_trajectory_cmd():
    rospy.init_node('trajectory_cmd_sender', anonymous=True)
    traj_pub = rospy.Publisher('/trajectory_cmd', String, queue_size=10)
    land_pub = rospy.Publisher('/land_cmd', Bool, queue_size=10)
    rate = rospy.Rate(1)

    signal.signal(signal.SIGINT, signal_handler)
    
    while not rospy.is_shutdown():
        try:
            cmd = input("Enter command (hover/circle/rectangle/eight/land/quit): ").strip().lower()
            if not cmd:
                rospy.logwarn("Empty input, please try again")
                continue
                
            if cmd in ['quit', 'q', 'exit']:
                rospy.loginfo("Exiting command sender...")
                break
            elif cmd == 'land':
                msg = Bool()
                msg.data = True
                land_pub.publish(msg)
                rospy.loginfo("Land command sent")
                break
            elif cmd in ['hover', 'circle', 'rectangle', 'eight']:
                msg = String()
                msg.data = cmd
                traj_pub.publish(msg)
                rospy.loginfo("Trajectory command sent: %s", cmd)
            else:
                rospy.logwarn("Invalid command: %s", cmd)
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

if __name__ == '__main__':
    try:
        send_trajectory_cmd()
    except rospy.ROSInterruptException:
        pass