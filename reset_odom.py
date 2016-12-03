def reset():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

    for i in range(0,10):
                pub.publish(Empty())
