import rospy
import xml.etree.ElementTree as ET
from rostak.cot_conversions import LLtoUTM
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path

class TakPath():

    def __init__(self):
        
        rospy.Subscriber('/tak/tak_rx', String, self.tak_tx_cb)

        self.path_utm_pub = rospy.Publisher("rostak_path_utm", Path, queue_size = 10)
        self.path_ll_pub = rospy.Publisher("rostak_path_ll", Path, queue_size = 10)
        self.remarks_pub = rospy.Publisher("rostak_path_remarks", String, queue_size = 10)


    def tak_tx_cb(self,msg):

        remarks = String()
        xml_f = ET.ElementTree(ET.fromstring(msg.data))
        root = xml_f.getroot()
        try:
            msg_type = root.attrib['type']
            if msg_type == 'b-m-r':
                details = root.findall('detail')
                remarks.data = details[0].find('remarks').text

                utm_msg = Path()
                utm_msg.header.stamp = rospy.Time.now()
                utm_msg.header.frame_id = 'utm'
                ll_msg = Path()
                ll_msg.header.stamp = rospy.Time.now()
                ll_msg.header.frame_id = 'latlon'
                waypoints = xml_f.findall("./detail/link")
                for wp in waypoints:
                    pnt_str = wp.attrib['point'].split(",")
                    (lat,lon) = (float(pnt_str[0]),float(pnt_str[1])) 
                    (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, lat, lon)

                    utm_pose = PoseStamped()
                    utm_pose.header = utm_msg.header
                    utm_pose.pose.position.x = float(crnt_utm_e)
                    utm_pose.pose.position.y = float(crnt_utm_n)
                    utm_pose.pose.orientation.w = 1
                    utm_msg.poses.append(utm_pose)

                    ll_pose = PoseStamped()
                    ll_pose.header = ll_msg.header
                    ll_pose.pose.position.x = lat
                    ll_pose.pose.position.y = lon
                    ll_pose.pose.orientation.w = 1
                    ll_msg.poses.append(ll_pose)
                #rospy.loginfo('Published Path Message: %s' %(utm_msg))
                self.path_utm_pub.publish(utm_msg)
                self.path_ll_pub.publish(ll_msg)
                self.remarks_pub.publish(remarks)
        except Exception as e:
            rospy.logwarn("Error of %s while parsing the path CoT xml" %(str(e)))

                

if __name__ == '__main__':
    
    rospy.init_node("rostak_path")
    tp = TakPath()
    rospy.spin()
