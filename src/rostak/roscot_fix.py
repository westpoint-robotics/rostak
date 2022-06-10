import rospy
import xml.etree.ElementTree as ET
import yaml
from rostak.functions import new_cot
from rostak.cotutil import CotUtil
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")

        config_path = rospy.get_param('~cot_config_path')
        self.util = CotUtil(config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.msg = String()
        rospy.Subscriber("fix", NavSatFix, self.publish_fix)
        rospy.loginfo(self.config)

    def publish_fix(self, msg):
        """Generate a location COT Event."""
        self.util.set_point(msg)
        cot = new_cot(self.config, 2 * max(1, 1 / self.rate))
        point = cot.find("point")
        point.set("lat", str(msg.latitude))
        point.set("lon", str(msg.longitude))
        point.set("hae", str(msg.altitude))
        self.msg.data = ET.tostring(cot).decode()
        self.tx.publish(self.msg)

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()