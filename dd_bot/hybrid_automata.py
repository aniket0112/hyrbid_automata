import numpy as np

# Configurable constants of the Hybrid Automata
L = 0.10 #Distance between two wheels
R = 0.05 #Radius of wheel

KP = 1
DISTANCE_TOL = 0.1 #meters
VELOCITY = 0.1 #meters
OBSTACLE_TOL = 0.5 #meters
FAT_GUARD_FACTOR = 0.3 #meters

# SlidingModeFlags
resetDistance = 100000
CLK = 0
CCLK = 1
SM_DIR = -1
class Position():
    def __init__(self,x,y,yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
    def mag(self):
        return np.sqrt(self.x**2+self.y**2)
class uniCycleState:
    def __init__(self,v,w):
        self.v = v
        self.w = w
class laserSensor:
    def __init__(self,mag,angle,size):
        self.mag = mag
        self.angle = angle
        self.size = size

def diffDriveKinematics(error,distance):                                        # Conversion of unicycle model to differential RPMs
    global KP,DISTANCE_TOL
    robot = uniCycleState(0,0)
    robot.w = KP*error
    if distance < DISTANCE_TOL:
        robot.v = 0
        robot.w = 0
    else:
        robot.v = VELOCITY
    rpmLeft = (2*robot.v-robot.w*L)/(2*R)
    rpmRight = (2*robot.v+robot.w*L)/(2*R)
    return (robot,rpmLeft,rpmRight,distance)
def angleBetweenTwoVector(a_,b_):                                               # Signed angle between two vectors
    a = Position(np.cos(a_),np.sin(a_))
    b = Position(np.cos(b_),np.sin(b_))
    angle = np.arccos((a.x*b.x+a.y*b.y)/(a.mag()*b.mag()))
    crossproduct = a.x*b.y-a.y*b.x
    if crossproduct > 0:
        return angle
    else:
        return -angle
def GTG(currPosition,desiredPosition):                                          # Mode: Go To Goal
    desiredHeading = np.arctan2(desiredPosition.y-currPosition.y,
                                desiredPosition.x-currPosition.x)
    distance = np.sqrt(np.square(desiredPosition.y-currPosition.y)
                      +np.square(desiredPosition.x-currPosition.x))
    error = angleBetweenTwoVector(currPosition.yaw,desiredHeading)
    return diffDriveKinematics(error,distance)
def OA(currPosition,avoidanceAngle):                                            # Mode: Obstacle Avoidance
    return diffDriveKinematics(angleBetweenTwoVector(currPosition.yaw,avoidanceAngle),10)
def SM(currPosition,desiredPosition,obstacleAngle):                             # Mode: Wall Follow
    global SM_DIR
    G2G_vector = Position(desiredPosition.x-currPosition.x,
                          desiredPosition.y-currPosition.y)
    CW_ObstacleAngle = obstacleAngle-1.57+currPosition.yaw
    CCW_ObstacleAngle = obstacleAngle+1.57+currPosition.yaw
    G2G_angle = np.arctan2(G2G_vector.y,G2G_vector.x)
    CW_vector = Position(np.cos(CW_ObstacleAngle),
                                    np.sin(CW_ObstacleAngle))
    CCW_vector = Position(np.cos(CCW_ObstacleAngle),
                                     np.sin(CCW_ObstacleAngle))
    if SM_DIR is -1:
        if np.dot([G2G_vector.x,G2G_vector.y],[CW_vector.x,CW_vector.y]) > 0:
            SM_DIR = CLK
#            print('CLK: '+str(np.rad2deg(CW_ObstacleAngle)))
            return diffDriveKinematics(angleBetweenTwoVector(currPosition.yaw,CW_ObstacleAngle),10)
        elif np.dot([G2G_vector.x,G2G_vector.y],[CCW_vector.x,CCW_vector.y]) > 0:
            SM_DIR = CCLK
#            print('CCLK: '+str(np.rad2deg(CCW_ObstacleAngle)))
            return diffDriveKinematics(angleBetweenTwoVector(currPosition.yaw,CCW_ObstacleAngle),10)
        else:
            return OA(currPosition,ObstacleAngle)
    elif SM_DIR is CLK:
#        print('CLK: '+str(np.rad2deg(CW_ObstacleAngle)))
        return diffDriveKinematics(angleBetweenTwoVector(currPosition.yaw,CW_ObstacleAngle),10)
    elif SM_DIR is CCLK:
#        print('CCLK: '+str(np.rad2deg(CCW_ObstacleAngle)))
        return diffDriveKinematics(angleBetweenTwoVector(currPosition.yaw,CCW_ObstacleAngle),10)
    else:
        SM_DIR = -1
        return OA(currPosition,ObstacleAngle)
def findObstacleAngle(obstacle_sense):                                          # Conversion of LaserScan Message to Obstacle Angle
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
    return (min,np.arctan2(obstaclePosition.y,obstaclePosition.x))

def switch(currPosition,desiredPosition,obstacle_sense):                        # Switch the controls modes
    global resetDistance, SM_DIR
    travelDistance = np.sqrt(np.square(desiredPosition.y-currPosition.y)
                            +np.square(desiredPosition.x-currPosition.x))
    print('SM_DIR: '+str(SM_DIR))
    if obstacle_sense.size is not 0:
        (distance,obstacleAngle) = findObstacleAngle(obstacle_sense)
        avoidanceAngle = np.arctan2(np.sin(obstacleAngle+3.14+currPosition.yaw),
                                    np.cos(obstacleAngle+3.14+currPosition.yaw))
        if distance > OBSTACLE_TOL+FAT_GUARD_FACTOR and travelDistance < resetDistance:
            print('Obstacle : YES : GTG')
            resetDistance = travelDistance
            SM_DIR = -1
            return GTG(currPosition,desiredPosition)                            #Do Go To GOAL
        elif distance <= OBSTACLE_TOL:
            print('Obstacle : YES : OA')
            return OA(currPosition,avoidanceAngle)
        else:
            print('Obstacle : YES : SM')
            return SM(currPosition,desiredPosition,obstacleAngle)
    else:
        print('Obstacle : NO : GTG')
        resetDistance = travelDistance
        SM_DIR = -1
        return GTG(currPosition,desiredPosition)
