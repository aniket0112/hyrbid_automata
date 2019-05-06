#!/usr/bin/python
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
pub_topic_l = '/dd_bot/left_wheel_controller/command'
pub_topic_r = '/dd_bot/right_wheel_controller/command'
sub_topic = '/gazebo/model_states'
sub_topic_laser = '/dd_bot/laser/scan'
L = 0.10
R = 0.05

x = 0
y = 0
head = 0
goal_x = 0
goal_y = 0
E = 0
error_old = 0

KP = 1
KI = 0
KD = 0

def readOdom(msg):
	global x,y,head
	x = msg.pose[1].position.x
	y = msg.pose[1].position.y
	rot = msg.pose[1].orientation
	(_,_,head) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])

def readLaser(msg):
	data = msg.ranges
	angle = []
	for i in range(len(data)):
		if data[i] != float('inf') and data[i] != -float('inf'):
			angle.append(i*180/720-90)
	print(angle)

controller = rospy.init_node('G2G_Controller',anonymous=True)
sub = rospy.Subscriber(sub_topic,ModelStates,readOdom)
sub_laser = rospy.Subscriber(sub_topic_laser,LaserScan,readLaser)
pub_l = rospy.Publisher(pub_topic_l,Float64,queue_size=10)
pub_r = rospy.Publisher(pub_topic_r,Float64,queue_size=10)
r = rospy.Rate(1)
rpm_r = Float64()
rpm_l = Float64()

while not rospy.is_shutdown():
	desired_head = np.arctan2((goal_y-y),(goal_x-x))
	error = desired_head-head
	E = E+error
	ED = error-error_old
	error_old = error
	W = KP*error+KI*E+KD*ED

	dist = np.sqrt(np.square(goal_y-y)+np.square(goal_x-x))
	if dist > 0.1:
		V = 0.5
	else:
		V = 0
		W = 0
		pub_l.publish(0)
		pub_r.publish(0)
#		break
	rpm_l = (2*V-W*L)/(2*R)
	rpm_r = (2*V+W*L)/(2*R)
#	print(np.rad2deg(desired_head),np.rad2deg(head),x,y)
	pub_l.publish(rpm_l)
	pub_r.publish(rpm_r)
	r.sleep()
