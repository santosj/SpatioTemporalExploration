#! /usr/bin/env python

import rospy
import math
# Brings in the SimpleActionClient
import geometry_msgs.msg
import spatiotemporalexploration.srv
import PyKDL


def pose_dist(pos1,pos2):
    dist = math.sqrt((pos1.position.x - pos2.position.x)**2 + (pos1.position.y - pos2.position.y)**2 + (pos1.position.z - pos2.position.z)**2)
    a = PyKDL.Rotation.Quaternion(pos1.orientation.x, pos1.orientation.y, pos1.orientation.z, pos1.orientation.w)
    b = PyKDL.Rotation.Quaternion(pos2.orientation.x, pos2.orientation.y, pos2.orientation.z, pos2.orientation.w)
    c = a * b.Inverse()
    ang= c.GetEulerZYX()
    return dist,ang[0]


class insert_pose_server(object):
    
    def __init__(self) :
               
        rospy.on_shutdown(self._on_node_shutdown)
        self.pose_pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, latch=True, queue_size=1)
        self.inj_pos_srv=rospy.Service('/inject_pose', spatiotemporalexploration.srv.InjectPose, self.inject_pose_cb)
        

    def inject_pose_cb(self, req):
        dist=10.0
        while dist>0.001:
            count=0
            while dist>0.001 and count<=20:
                print 'injecting_pose try: %d' %count
                new_pos=geometry_msgs.msg.PoseWithCovarianceStamped()
                new_pos.header.stamp = rospy.Time.now()
                new_pos.pose.pose = req.pose
                self.pose_pub.publish(new_pos)
                rospy.sleep(0.1)
                
                #rec_pos=rospy.wait_for_message('robot_pose', geometry_msgs.msg.PoseWithCovarianceStamped)
                rec_pos=rospy.wait_for_message('robot_pose', geometry_msgs.msg.Pose)
                print rec_pos
                dist,ang = pose_dist(new_pos.pose.pose, rec_pos)
                print "DIST:",dist
                print "Ang:",ang

            if dist>0.001:
                new_pos=geometry_msgs.msg.PoseWithCovarianceStamped()
                new_pos.header.stamp = rospy.Time.now()
                new_pos.pose.pose.position.x = 0.0
                new_pos.pose.pose.position.y = 0.0
                new_pos.pose.pose.position.z = 0.001
                new_pos.pose.pose.orientation.x=0.0
                new_pos.pose.pose.orientation.y=0.0
                new_pos.pose.pose.orientation.z=0.0
                new_pos.pose.pose.orientation.w=1.0
                self.pose_pub.publish(new_pos)
                rospy.sleep(0.1)
                               
        return True
        


    def _on_node_shutdown(self):
        print "Adios Amigos"
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('inject_pose_server')
    ps = insert_pose_server()
    rospy.spin()
