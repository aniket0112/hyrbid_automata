import numpy as np
L = 0.10 #Distance between two wheels
R = 0.05 #Radius of wheel

KP = 0.5
DISTANCE_TOL = 0.1
VELOCITY = 0.3
OBSTACLE_TOL = 0.7
FAT_GUARD_FACTOR = 0.3

resetDistance = 0

class Vector:
  def __init__(self,point):
    self.x = point[0]
    self.y = point[1]
  def cart(self)
    return np.array([self.x,self.y])
  def polar(self)
    return np.array([np.sqrt(self.x**2+self.y**2),np.arctan2(self.y,self.x)])
class uniCycleState:
    def __init__(self,v,w):
        self.v = v
        self.w = w
class laserSensor:
    def __init__(self,mag,angle,size):
        self.mag = mag
        self.angle = angle
        self.size = size

def angleBetweenTwoVector(A,B):
    a = A.cart()
    b = B.cart()
    [mag_a,_]=A.polar()
    [mag_b,_]=B.polar()
    angle = np.arccos(np.dot(a,b)/(mag_a*mag_b))
    crossproduct = A.x*B.y-A.y*B.x
    if crossproduct > 0:
        return angle
    else:
        return -angle

def G2G(goal_vector,position_vector,robot_yaw_vector):
    global KP,DISTANCE_TOL
    robot = uniCycleState(0,0)
    u_G2G = Vector(goal_vector.cart() - position_vector.cart())
    (distance,phi) = u_G2G.polar()
    error = angleBetweenTwoVector(robot_yaw_vector,u_G2G)
    robot.w = KP*error
    if distance < DISTANCE_TOL:
        robot.v = 0
        robot.w = 0
    else:
        robot.v = VELOCITY
    rpmLeft = (2*robot.v-robot.w*L)/(2*R)
    rpmRight = (2*robot.v+robot.w*L)/(2*R)
    return (robot,rpmLeft,rpmRight,distance)

#Give obstacle position vector wrt ROBOT FRAME
def findObstaclePostion(obstacle_sense):
    obstaclePosition = Vector(0,0)
    min = 100000
    for i in range(len(obstacle_sense.mag)):
        if obstacle_sense.mag[i] is not 0:
            if min > obstacle_sense.mag[i]:
                min = obstacle_sense.mag[i]
            w = [(1/obstacle_sense.mag[i])*np.cos(obstacle_sense.angle[i]),
                (1/obstacle_sense.mag[i])*np.sin(obstacle_sense.angle[i])]
            obstaclePosition.x = obstaclePosition.x + w[0]
            obstaclePosition.y = obstaclePosition.y + w[1]
    return (min,obstaclePosition)

def switch(currPosition, desiredPosition, obstacle_sense):
