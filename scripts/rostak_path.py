import rospy
import xml.etree.ElementTree as ET
from rostak.LatLongUTMconversion import LLtoUTM
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path

class TakPath():

    def __init__(self):
        
        rospy.Subscriber('/tak/tak_rx', String, self.tak_tx_cb)

        self.path_pub = rospy.Publisher("rostak_path", Path, queue_size = 10)
        self.remarks_pub = rospy.Publisher("rostak_path_remarks", String, queue_size = 10)



    def tak_tx_cb(self,msg):

        remarks = String()
        xml_f = ET.ElementTree(ET.fromstring(msg.data))
        root = xml_f.getroot()
#        try:
        msg_type = root.attrib['type']
        if msg_type == 'b-m-r':
            details = root.findall('detail')
            remarks.data = details[0].find('remarks').text

            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = 'utm'
            waypoints = xml_f.findall("./detail/link")
            for wp in waypoints:
                pnt_str = wp.attrib['point'].split(",")
                (lat,lon) = (float(pnt_str[0]),float(pnt_str[1])) 
                (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, lat, lon)

                wp_pose = PoseStamped()
                wp_pose.header = path_msg.header
                wp_pose.pose.position.x = float(crnt_utm_e)
                wp_pose.pose.position.y = float(crnt_utm_n)
                wp_pose.pose.orientation.w = 1
                path_msg.poses.append(wp_pose)
            rospy.loginfo('Published Path Message: %s' %(path_msg))
            self.path_pub.publish(path_msg)
            self.remarks_pub.publish(remarks)
#       except Exception as e:
#            rospy.logwarn("Error of %s while parsing the path CoT xml" %(str(e)))

                


if __name__ == '__main__':
    
    rospy.init_node("rostak_path")
    tp = TakPath()
    rospy.spin()
