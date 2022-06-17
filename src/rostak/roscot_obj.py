#!/usr/bin/env python3
import rospy
from rostak.cot_utils import CotUtility
from rostak.cot_conversions import UTMtoLL
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

class RosCotObj:
    def __init__(self):
        rospy.init_node("roscot_obj")
        self.config_path = rospy.get_param('~cot_params')
        self.util = CotUtility(self.config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.tx_msg = String()
        self.zone = self.util.get_zone()
        rospy.Subscriber("obj", MarkerArray, self.publish_obj)
        rospy.loginfo(self.util.get_config())

    def publish_obj(self, msg):
        for marker in msg.markers:
            self.util.set_object_str(marker.text)
            if marker.header.frame_id == 'utm':
                (lat,lon) = UTMtoLL(23,marker.pose.position.y,marker.pose.position.x,self.zone)
            else:
                lat, lon = marker.pose.position.x ,marker.pose.position.y

            self.util.set_point((lat,lon,marker.pose.position.z))
            stale_in = 2 * max(1, 1 / self.rate)
            self.tx_msg.data = self.util.new_status_msg(stale_in)
            self.tx.publish(self.tx_msg)

if __name__ == '__main__':
    RosCotObj()
    rospy.spin()