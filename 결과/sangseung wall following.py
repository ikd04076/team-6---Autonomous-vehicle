#!/usr/bin/env pytho
 
import rospy 
from ackermann_msgs.msg import AckermannDriveStamped 
from sensor_msgs.msg import LaserScan

msgs = AckermannDriveStamped() 
msgs.drive.speed=1
msgs.drive.steering_angle=0 


def talker(scan_msg):
    go = scan_msg.ranges[540]
    right = scan_msg.ranges[320]
    left = scan_msg.ranges[760]
    rightside = scan_msg.ranges[270]
    leftside = scan_msg.ranges[810]
    print(go)


    if go<4 :
        msgs.drive.speed=0.5
        msgs.drive.steering_angle=0
        if right<left:
            msgs.drive.speed=3
            msgs.drive.steering_angle=1.5
        elif left<right:
            msgs.drive.speed=3
            msgs.drive.steering_angle=-1.5

    elif rightside < 0.2:
        msgs.drive.speed=0.03
        msgs.drive.steering_angle=2
    elif leftside < 0.2:
        msgs.drive.speed=0.03
        msgs.drive.steering_angle=-2
    else :
        msgs.drive.speed=5
        msgs.drive.steering_angle=0

 


               
                    


    


    








pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
rospy.init_node('wall-following')
rospy.Subscriber('scan',LaserScan, talker, queue_size = 10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    pub.publish(msgs)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
