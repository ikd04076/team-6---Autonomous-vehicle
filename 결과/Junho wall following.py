#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
 
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


def talker(scan_msg):
    
    A = scan_msg.ranges[270]
    B = scan_msg.ranges[405]
    C = scan_msg.ranges[540]
    D = scan_msg.ranges[675]
    E = scan_msg.ranges[810] 
    if A<1:
        msgs.drive.speed=1
        msgs.drive.steering_angle=0.5
    elif A<0.5:
        msgs.drive.speed=.5
        msgs.drive.steering_angle=1
    elif B<2:
        msgs.drive.speed=1
        msgs.drive.steering_angle=0.3
    elif B<1:        
        msgs.drive.speed=1
        msgs.drive.steering_angle=0.5
    elif C<2:
        msgs.drive.speed=1
        msgs.drive.steering_angle=-0.3
    elif D<2:
        msgs.drive.speed=2
        msgs.drive.steering_angle=-0.3
    elif D<1:        
        msgs.drive.speed=1
        msgs.drive.steering_angle=-0.5
    elif E<1:
        msgs.drive.speed=1
        msgs.drive.steering_angle=-0.5
    elif E<0.5:  
        msgs.drive.speed=.5
        msgs.drive.steering_angle=-1
    else:
        msgs.drive.speed=3
        msgs.drive.steering_angle=0
    print(A)
    print(B)
    print(C)
    print(D)
    print(E)

msgs = AckermannDriveStamped()
    
pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
rospy.init_node('go')
rospy.Subscriber('scan', LaserScan, talker, queue_size = 10)
rate = rospy.Rate(10) 
    
while not rospy.is_shutdown():

    pub.publish(msgs)
    rate.sleep()

if __name__ == '__main__':
    try:
        ()
    except rospy.ROSInterruptException:
        pass
