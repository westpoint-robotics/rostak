import rospy
import numpy as np
from tf2_ros import tf_conversions
import xml.etree.ElementTree as ET
from rostak.LatLongUTMconversion import LLtoUTM
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped


class TakGoTo():

    def __init__(self):
        
        rospy.Subscriber('/tak/tak_rx', String, self.tak_tx_cb)

        self.goto_pub = rospy.Publisher("rostak_goto", PoseStamped, queue_size = 10)

    def tak_tx_cb(self,msg):
        xml_f = ET.ElementTree(ET.fromstring(msg.data))
        root = xml_f.getroot()
        
        if root.attrib['type'] == 'b-m-p-w-GOTO':
            lat = float(xml_f.find("./point").attrib['lat'])
            lon = float(xml_f.find("./point").attrib['lon'])
            remarks = xml_f.find("./detail").find("./remarks").text.split(',')
            tak_orientation, tak_altitude = remarks[2], remarks[3]

            (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, lat, lon)

            pose_msg = PoseStamped()
            if tak_orientation != 'NA':
                quat = tf_conversions.transformations.quaternion_from_euler(0,0,float(tak_orientation/180*np.pi))
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

            if tak_altitude != 'NA':
                pose_msg.pose.position.z = float(tak_altitude)

            pose_msg.pose.position.x = crnt_utm_e
            pose_msg.pose.position.y = crnt_utm_n
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'utm'

            self.goto_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node("rostak_goto")
    TakGoTo()
    rospy.spin()