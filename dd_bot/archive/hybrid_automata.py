import numpy as np
L = 0.10 #Distance between two wheels
R = 0.05 #Radius of wheel

KP = 0.5
DISTANCE_TOL = 0.1
VELOCITY = 0.3
OBSTACLE_TOL = 0.7
FAT_GUARD_FACTOR = 0.3

resetDistance = 0

class Position:
    def __init__(self,x,y,yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
    def mag(self):
        return np.sqrt(self.x**2+self.y**2)
    def phi(self):
        return np.arctan2(self.y,self.x)

class uniCycleState:
    def __init__(self,v,w):
        self.v = v
        self.w = w

class laserSensor:
    def __init__(self,mag,angle,size):
        self.mag = mag
        self.angle = angle
        self.size = size

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def angleBetweenTwoVector(a_,b_):
    a = Position(np.cos(a_),np.sin(a_))
    b = Position(np.cos(b_),np.sin(b_))
    angle = np.arccos((a.x*b.x+a.y*b.y)/(a.mag()*b.mag()))
    crossproduct = a.x*b.y-a.y*b.x
    if crossproduct > 0:
        return angle
    else:
        return -angle

def robotToWorldTransform(currPosition, objectPosition):
    [x,y] = pol2cart(objectPosition.mag(),objectPosition.phi()+currPosition.yaw)
    return Position(x,y)

def GTG(currPosition,desiredPosition):
    global KP,DISTANCE_TOL
    robot = uniCycleState(0,0)
    desiredHeading = np.arctan2(desiredPosition.y-currPosition.y,
                                desiredPosition.x-currPosition.x)
    distance = np.sqrt(np.square(desiredPosition.y-currPosition.y)
                      +np.square(desiredPosition.x-currPosition.x))
    error = angleBetweenTwoVector(currPosition.yaw,desiredHeading)
    print(np.rad2deg([desiredHeading,currPosition.yaw]))
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
    obstaclePosition = Position(0,0)
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

def OA(currPosition,obstaclePosition):
    OA_vector = Position(-obstaclePosition.x,-obstaclePosition.y)
    OA_vector = robotToWorldTransform(currPosition,obstaclePosition)
    print(str(np.rad2deg(OA_vector.phi()))+' Obstacle Avoidance Angle')
    return GTG(currPosition,OA_vector)

def slidingMode(currPosition,desiredPosition,obstaclePosition):
    G2G_vector = Position(desiredPosition.x-currPosition.x,
                        desiredPosition.y-currPosition.y)
    slidingModeCW_vector = Position(-(obstaclePosition.y),
                        obstaclePosition.x)
    slidingModeCCW_vector = Position(obstaclePosition.y,
                        -(obstaclePosition.x))

    if np.dot([G2G_vector.x,G2G_vector.y],
                [slidingModeCW_vector.x,slidingModeCW_vector.y]) > 0:
        #print('Clockwise Sliding Mode control')
        return GTG(currPosition,robotToWorldTransform(currPosition,slidingModeCW_vector))
    elif np.dot([G2G_vector.x,G2G_vector.y],
                [slidingModeCCW_vector.x,slidingModeCCW_vector.y]) > 0:
        #print('Counterclockwise Sliding Mode control')
        return GTG(currPosition,robotToWorldTransform(currPosition,slidingModeCCW_vector))
    else:
        return OA(currPosition,obstaclePosition)

def switch(currPosition, desiredPosition, obstacle_sense):
    global resetDistance
    travelDistance = np.sqrt(np.square(desiredPosition.y-currPosition.y)
                      +np.square(desiredPosition.x-currPosition.x))
    if obstacle_sense.size is not 0 :
        (distance,obstacle_vector) = findObstaclePostion(obstacle_sense)
        if distance > OBSTACLE_TOL+FAT_GUARD_FACTOR and travelDistance < resetDistance:
            print('ObstacleAngle: '+str(np.rad2deg(np.arctan2(obstacle_vector.y,obstacle_vector.x)))+' GTG')
            resetDistance = travelDistance
            return GTG(currPosition,desiredPosition)                                              #Do Go To GOAL
        elif distance <= OBSTACLE_TOL:
            print('ObstacleAngle: '+str(np.rad2deg(np.arctan2(obstacle_vector.y,obstacle_vector.x)))+' OA')               #Do Obstacle Avoidance
            return OA(currPosition,obstacle_vector)
        else:
            print('ObstacleAngle: '+str(np.rad2deg(np.arctan2(obstacle_vector.y,obstacle_vector.x)))+' SM')
            return slidingMode(currPosition,desiredPosition,obstacle_vector)
    else:
        #print('No Obstacle: GTG')
        resetDistance = travelDistance
        return GTG(currPosition,desiredPosition)
