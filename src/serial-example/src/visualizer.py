#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
import tf

attitude = 'none'
gps1 = 'none'
gps2 = 'none'
delta_lat = 1;
delta_lon = 1;
pi = 3.14159265

def calc_distance(gps1, gps2):
    gps1_lat = round(gps1.latitude, 6)
    gps1_lon = round(gps1.longitude, 6)
    gps2_lat = round(gps2.latitude, 6)
    gps2_lon = round(gps2.longitude, 6)
    global delta_lat, delta_lon
    delta_lat = (gps1_lat*(108000) - gps2_lat*(108000))
    delta_lon = (gps1_lon*(108000) - gps2_lon*(108000))
    hyp_m = (delta_lat**2 + delta_lon**2)**0.5
    hyp_ft = (hyp_m*3.2800839)
    rospy.loginfo("Distance is %s in ft.", hyp_ft)

def gpscallback(fix):
    gps2 = fix
    gps1 = NavSatFix()
    gps1.latitude = 39.4333983
    gps1.longitude = -77.418547
    calc_distance(gps1,gps2)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d %d %d", data.x, data.y, data.z)
    attitude = data
    br = tf.TransformBroadcaster()
    br.sendTransform((delta_lat, delta_lon, 0),
                     tf.transformations.quaternion_from_euler(attitude.x*pi/180, attitude.y*pi/180, attitude.z*pi/180),
                     rospy.Time.now(),
                     'UAV',
                     "world")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('visualizer', anonymous=True)

    rospy.Subscriber("attitude", Vector3, callback)
    rospy.Subscriber("fix", NavSatFix, gpscallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()