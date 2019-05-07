#!/usr/bin/python
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import hybrid_automata as HA

pub_topic_l = '/dd_bot/left_wheel_controller/command'
pub_topic_r = '/dd_bot/right_wheel_controller/command'
sub_topic = '/gazebo/model_states'
sub_topic_laser = '/dd_bot/laser/scan'
L = 0.10
R = 0.05

currPosition = HA.Position(0,0,0)
desiredPosition = HA.Position(5,5,0)
laser_data = HA.laserSensor([],[],0)

def readOdom(msg):
	global currPosition
	currPosition.x = msg.pose[1].position.x
	currPosition.y = msg.pose[1].position.y
	rot = msg.pose[1].orientation
	(_,_,currPosition.yaw) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])

def readLaser(msg):
	global laser_data
	data = msg.ranges
	mag = []
	angle = []
	for i in range(len(data)):
		if data[i] != float('inf') and data[i] != -float('inf'):
			angle.append(i*6.28/720-3.14)
			mag.append(data[i])
	laser_data.mag = mag
	laser_data.angle = angle
	laser_data.size = len(angle)

controller = rospy.init_node('G2G_Controller',anonymous=True)
sub = rospy.Subscriber(sub_topic,ModelStates,readOdom)
sub_laser = rospy.Subscriber(sub_topic_laser,LaserScan,readLaser)
pub_l = rospy.Publisher(pub_topic_l,Float64,queue_size=10)
pub_r = rospy.Publisher(pub_topic_r,Float64,queue_size=10)
r = rospy.Rate(1)

while not rospy.is_shutdown():
	(_,rpmLeft,rpmRight,distance) = HA.switch(currPosition,desiredPosition,laser_data)
	pub_l.publish(rpmLeft)
	pub_r.publish(rpmRight)
#	print(currPosition.x,currPosition.y)
	r.sleep()
