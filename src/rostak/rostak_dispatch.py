import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path

class RosTakDispatch:
    def __init__(self):
        rospy.init_node("roscot_dispatch")
        rospy.Subscriber("tak_cmd", String, self.handle_cmd)
        self.pub_path = rospy.Publisher("tak_goto", Path, queue_size=1)
        self.pub_dr = rospy.Publisher("tak_deadreckon", String, queue_size=1)
        self.id_q = Quaternion()
        self.id_q.x = 0
        self.id_q.y = 0
        self.id_q.z = 0
        self.id_q.w = 1

    def handle_cmd(self, msg):
        cmd = msg.data.split(" ")
        if cmd[0] == "#!goto":
            self.send_path(cmd[2:])
        elif cmd[0] == "#!dr":
            self.send_deadreckon(cmd[2:])
        # elif cmd[0] == "#!explore":
        # elif cmd[0] == "#!snap":
        # elif cmd[0] == "#!bag":
        # elif cmd[0] == "#!video":
        else:
            rospy.loginfo("RosTakDispatcher does not support command %s", cmd[0])

    def send_path(self, coords):
        rospy.loginfo(coords)
        path = Path()
        path.poses = []
        for coord in coords:
            part = coord.split(",")
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = rospy.Time.now()
            pose.pose = Pose()
            pose.pose.position = Point()
            pose.pose.position.x = float(part[0])
            pose.pose.position.y = float(part[1])
            pose.pose.position.z = float(part[2])
            pose.pose.orientation = self.id_q
            path.poses.append(pose)
        self.pub_path.publish(path)

    def send_deadreckon(self, data):
        msg = String()
        msg.data = string.join(" ".join(data))
        self.pub_dr.publish(msg)

if __name__ == "__main__":
    client = RosTakDispatch()
    rospy.spin()
