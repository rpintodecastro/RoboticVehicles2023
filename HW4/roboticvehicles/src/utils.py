#!/usr/bin/env python
# license removed for brevity
import rospy

import tf

from roboticvehicles.msg import vehicle2Dpose


br = tf.TransformBroadcaster()


#*********************************************************
# republishes vehicle position and orientation message using tf
def callback_vehicle2Dpose(msg):
    br.sendTransform( (msg.x, msg.y, 0),
                       tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                       rospy.Time.now(),
                       "base_link",
                        "world")    
def main():
    rospy.init_node('utils', anonymous=True)

    # initializes vehicle position
    br.sendTransform( (0, 0, 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    "base_link",
                    "world")    

    rospy.Subscriber("vehicle2Dpose", vehicle2Dpose, callback_vehicle2Dpose)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()