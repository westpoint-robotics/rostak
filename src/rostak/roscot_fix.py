import rospy
from rostak.cot_utility import CotUtility
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")
        config_path = rospy.get_param('~cot_params')
        self.util = CotUtility(config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.msg = String()
        rospy.Subscriber("fix", NavSatFix, self.publish_fix)
        rospy.loginfo(self.util.get_config())

    def publish_fix(self, msg):
        """Generate a status COT Event."""
        self.util.set_point(msg)
        stale_in = 2 * max(1, 1 / self.rate)
        self.msg.data = self.util.new_status_msg(stale_in)
        self.tx.publish(self.msg)

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()