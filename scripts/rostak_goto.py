import rospy
import numpy as np
import tf2_ros as tf
import xml.etree.ElementTree as ET
from rostak.cot_conversions import LLtoUTM
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped


class TakGoTo():

    def __init__(self):
        
        rospy.Subscriber('/tak/tak_rx', String, self.tak_rx_cb)

        self.goto_utm_pub = rospy.Publisher("rostak_goto_utm", PoseStamped, queue_size = 10)
        self.goto_ll_pub = rospy.Publisher("rostak_goto_ll", PoseStamped, queue_size = 10)

    def tak_rx_cb(self,msg):
        xml_f = ET.ElementTree(ET.fromstring(msg.data))
        root = xml_f.getroot()
        
        if root.attrib['type'] == 'b-m-p-w-GOTO':
            lat = float(xml_f.find("./point").attrib['lat'])
            lon = float(xml_f.find("./point").attrib['lon'])
            remarks = xml_f.find("./detail").find("./remarks").text.split(',')
            tak_orientation, tak_altitude = remarks[2], remarks[3]

            (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, lat, lon)

            utm_msg = PoseStamped()
            ll_msg = PoseStamped()
            if tak_orientation != 'NA':
                quat = tf.tf_conversions.transformations.quaternion_from_euler(0,0,float(tak_orientation/180*np.pi))
                utm_msg.pose.orientation.x = quat[0]
                utm_msg.pose.orientation.y = quat[1]
                utm_msg.pose.orientation.z = quat[2]
                utm_msg.pose.orientation.w = quat[3]

                ll_msg.pose.orientation.x = quat[0]
                ll_msg.pose.orientation.y = quat[1]
                ll_msg.pose.orientation.z = quat[2]
                ll_msg.pose.orientation.w = quat[3]

            if tak_altitude != 'NA':
                utm_msg.pose.position.z = float(tak_altitude)
                ll_msg.pose.position.z = float(tak_altitude)

            utm_msg.pose.position.x = crnt_utm_e
            utm_msg.pose.position.y = crnt_utm_n
            utm_msg.header.stamp = rospy.Time.now()
            utm_msg.header.frame_id = 'utm'

            ll_msg.pose.position.x = lat
            ll_msg.pose.position.y = lon
            ll_msg.header.stamp = rospy.Time.now()
            ll_msg.header.frame_id = 'latlon'
            print('POSE MSG: ', utm_msg)

            self.goto_utm_pub.publish(utm_msg)
            self.goto_ll_pub.publish(ll_msg)

if __name__ == '__main__':
    rospy.init_node("rostak_goto")
    TakGoTo()
    rospy.spin()