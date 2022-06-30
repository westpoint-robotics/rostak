import rospy
from rostak.cot_utility import CotUtility
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class RosTakClient:
    def __init__(self):
        rospy.init_node("roscot_client")
        config_path = rospy.get_param("~cot_params")
        self.rate = rospy.get_param("~rate", 0.2)
        self.pub_cmd = rospy.Publisher("tak_cmd", String, queue_size=1)
        self.pub_tx = rospy.Publisher("tak_tx", String, queue_size=1)
        self.status_msg = String()
        self.cmd_msg = String()
        self.util = CotUtility(config_path)
        self.fixed = False
        self.status_count = 0
        rospy.Subscriber("tak_rx", String, self.handle_cot)
        rospy.Subscriber("tak_chat", String, self.handle_chat)
        rospy.Subscriber("fix", NavSatFix, self.handle_fix)

    def handle_cot(self, msg):
        """Process cot received from tak"""
        self.cmd_msg.data = self.util.process_cot(msg.data)
        if self.cmd_msg.data:
            self.pub_cmd.publish(self.cmd_msg)

    def handle_chat(self, msg):
        """Publish a chat from ros to tak."""
        self.chat_msg.data = self.util.new_chat_msg(msg.data)
        self.pub_tx.publish(self.chat_msg)

    def handle_fix(self, msg):
        """update local fix"""
        self.fixed = True
        self.util.set_point(msg)

    def handle_speed(self, msg):
        """ update local speed """
        self.util.set_speed(0)

    def handle_heading(self, msg):
        self.util.set_course(0.0)

    def publish_status(self): 
        """Publish a status cot to tak."""
        
        # wait for initial fix
        if not self.fixed:
            return
        
        # process every 30s (stationary) or 1s if moving
        if self.status_count == 0 or self.util.is_moving():
            self.status_msg.data = self.util.new_status_msg()
            self.pub_tx.publish(self.status_msg)
        self.status_count = (self.status_count + 1) % 30

if __name__ == "__main__":
    client = RosTakClient()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        client.publish_status()
        rate.sleep()
