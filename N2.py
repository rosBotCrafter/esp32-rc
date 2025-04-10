#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import Float32

class PublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('node2', anonymous=True)

        # Publisher that sends data
        self.publisher = rospy.Publisher('pb2', Float32, queue_size=10)

        # Subscriber that listens to processed data
        rospy.Subscriber('pb1', Float32, self.subscriber_callback)

        # Publisher's data (initial integer value)
        self.data = 5
        # self.nwdata=0.0

    def subscriber_callback(self, data):
        # Callback function that updates the publisher's data based on subscriber's message
        rospy.loginfo("Publisher received data from subscriber: %s" % data.data)

        
        # self.nwdata=self.data
        self.data = self.data+0.1*(data.data-self.data)  # Updating publisher's data with subscriber's data
        self.publish_data()
    
             

    def publish_data(self):
        # Publish the data to the topic
        rospy.loginfo("Publisher publishing data: %s" % self.data)
        self.publisher.publish(self.data)

    def run(self):
        # Run the publisher node
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PublisherNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

