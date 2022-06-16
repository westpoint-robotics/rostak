import rospy
from rostak.cot_utils import CotUtility
from rostak.LatLongUTMconversion import UTMtoLL
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")
        config_path = rospy.get_param('~cot_params')
        self.util = CotUtility(config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.msg = String()
        rospy.Subscriber("fix_latlon", NavSatFix, self.publish_fix_ll)
        rospy.Subscriber("fix_utm", PointStamped, self.publsih_fix_utm)
        rospy.loginfo(self.util.get_config())

    def publish_fix_ll(self, msg):
        """Generate a status COT Event."""
        self.util.set_point((msg.latitude,msg.longitude,msg.altitude))
        stale_in = 2 * max(1, 1 / self.rate)
        self.msg.data = self.util.new_status_msg(stale_in)
        print(self.msg.data)
        self.tx.publish(self.msg)

    def publsih_fix_utm(self, msg):
        """ Generate status COT event from robot pose """
        (lat, lon) = UTMtoLL(23, msg.point.y, msg.point.x, self.zone)
        self.util.set_point((lat,lon,msg.point.z))
        stale_in = 2 * max(1, 1 / self.rate)
        self.msg.data = self.util.new_status_msg(stale_in)
        self.tx.publish(self.msg)
        

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()