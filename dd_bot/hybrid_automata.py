import numpy as np
L = 0.10 #Distance between two wheels
R = 0.05 #Radius of wheel

KP = 0.5
DISTANCE_TOL = 0.1
VELOCITY = 0.3
OBSTACLE_TOL = 75
FAT_GUARD_FACTOR = 10
class Position:
    def __init__(self,x,y,yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw

class uniCycleState:
    def __init__(self,v,w):
        self.v = v
        self.w = w

class laserSensor:
    def __init__(self,mag,angle,size):
        self.mag = mag
        self.angle = angle
        self.size = size

def GTG(currPosition, desiredPosition):
    global KP,DISTANCE_TOL
    robot = uniCycleState(0,0)
    desiredHeading = np.arctan2((desiredPosition.y-currPosition.y),
                                (desiredPosition.x-currPosition.x))
    distance = np.sqrt(np.square(desiredPosition.y-currPosition.y)
                      +np.square(desiredPosition.x-currPosition.x))
    error = desiredHeading - currPosition.yaw
    robot.w = KP*error
    if distance < DISTANCE_TOL:
        robot.v = 0
        robot.w = 0
    else:
        robot.v = VELOCITY
    rpmLeft = (2*robot.v-robot.w*L)/(2*R)
    rpmRight = (2*robot.v+robot.w*L)/(2*R)
    return (robot,rpmLeft,rpmRight,distance)

def findObstaclePostion(obstacle_sense):
    obstaclePosition = Position(0,0)
    for i in range(len(obstacle_sense.mag)):
        if obstacle_sense.mag[i] is not 0:
            w = [(1/obstacle_sense.mag[i])*np.cos(obstacle_sense.angle[i]),
                (1/obstacle_sense.mag[i])*np.sin(obstacle_sense.angle[i])]
            obstaclePosition.x = obstaclePosition.x + w[0]
            obstaclePosition.y = obstaclePosition.y + w[1]
    return obstaclePosition

def OA(currPosition, desiredPosition, obstaclePosition):
    G2G_vector = Position(desiredPosition.x-currPosition.x,
                        desiredPosition.y-currPosition.y)
    OA_vector = Position(G2G_vector.x-obstaclePosition.x,
                        G2G_vector.y-obstaclePosition.y)
    return GTG(currPosition,OA_vector)

def switch(currPosition, desiredPosition, obstacle_sense):
    if obstacle_sense.size is not 0:
        print('Obstacle detected')
        obstaclePosition = findObstaclePostion(obstacle_sense)
        distance = np.sqrt(np.square(obstaclePosition.y-currPosition.y)
                          +np.square(obstaclePosition.x-currPosition.x))
        if distance <= OBSTACLE_TOL:
            print('Distance to obstacle: '+str(distance)+' : GTG')
            #Do Go To GOAL
            return GTG(currPosition,desiredPosition)
        elif distance > OBSTACLE_TOL+FAT_GUARD_FACTOR:
            #Do Obstacle Avoidance
            print('Distance to obstacle: '+str(distance)+' : OA')
            return OA(currPosition,desiredPosition,obstaclePosition)
    else:
        print('No Obstacle: GTG')
        return GTG(currPosition,desiredPosition)

#dummySensor = laserSensor([1.6,1.5,0.3,0.1,0.3,0.2,1.1],np.deg2rad([-90,-60,-30,0,30,60,90]),7)
#dummyPosition = findObstaclePostion(dummySensor)
#currPosition = Position(5,5,3.14/4)
#desiredPosition = Position(6,0)
