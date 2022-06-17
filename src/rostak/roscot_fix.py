#!/usr/bin/env python3
import rospy
from rostak.cot_utils import CotUtility
from rostak.cot_conversions import UTMtoLL
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")
        config_path = rospy.get_param('~cot_params')
        self.util = CotUtility(config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.tx_msg = String()
        self.zone = self.util.get_zone()
        rospy.Subscriber("fix", PointStamped, self.publish_fix)
        print('here here')
        rospy.loginfo(self.util.get_config())

    def publish_fix(self, msg):
        if msg.header.frame_id == 'utm':
            (lat,lon) = UTMtoLL(23, msg.point.y, msg.point.x, self.zone)
        else:
            lat,lon = msg.point.x, msg.point.y
        self.util.set_point((lat,lon,msg.point.z))
        stale_in = 2 * max(1, 1 / self.rate)
        self.tx_msg.data = self.util.new_status_msg(stale_in)
        self.tx.publish(self.tx_msg)
        

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()