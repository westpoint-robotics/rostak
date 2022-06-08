import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import xml.etree.ElementTree as ET
import yaml
from rostak.functions import new_cot

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")

        config_path = rospy.get_param('~cot_config_path')
        with open(config_path) as config:
            self.config = yaml.safe_load(config)
        rospy.loginfo(self.config)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.msg = String()
        rospy.Subscriber("fix", NavSatFix, self.handle_message)

    def handle_message(self, msg):
        """Generate a location COT Event."""
        cot = new_cot(self.config, 0.5)
        point = cot.find("point")
        point.set("lat", str(msg.latitude))
        point.set("lon", str(msg.longitude))
        point.set("hae", str(msg.altitude))
        self.msg.data = ET.tostring(cot).decode()
        self.tx.publish(self.msg)

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()