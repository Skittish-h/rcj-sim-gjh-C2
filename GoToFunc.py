import math
import time
#oprional argument
#pointToGoal = true it will point to goal else it does nothing 
#
"""git add gotofunc.py
git commit -m "msg for users"
git push"""
def get_angles(ball_pos: dict, robot_pos: dict):
        """Get angles in degrees.

        Args:
            ball_pos (dict): Dict containing info about position of the ball
            robot_pos (dict): Dict containing info about position and rotation
                of the robot

        Returns:
            :rtype: (float, float):
                Angle between the robot and the ball
                Angle between the robot and the north
        """
        robot_angle: float = robot_pos['orientation']

        # Get the angle between the robot and the ball
        angle = math.atan2(
            ball_pos['y'] - robot_pos['y'],
            ball_pos['x'] - robot_pos['x'],
        )

        if angle < 0:
            angle = 2 * math.pi + angle

        if robot_angle < 0:
            robot_angle = 2 * math.pi + robot_angle

        robot_ball_angle = math.degrees(angle + robot_angle)

        # Axis Z is forward
        # TODO: change the robot's orientation so that X axis means forward
        robot_ball_angle -= 90
        if robot_ball_angle > 360:
            robot_ball_angle -= 360

        return robot_ball_angle, robot_angle
def goTo(x, y, robot_pos, robot_angle, point_goal=False, should_soften = True, magicnum= 0.20, ball_pos={'x':0, 'y':0}, ignor = True):
    
    if ball_pos['x'] > 0.6 and ignor:
        DesiredPos = dict()
        x = 0.3
        y = 0.5
        DesiredPos['x'] = x
        DesiredPos['y'] = y
        if(math.sqrt((DesiredPos['x'] - robot_pos['x'])**2 + (DesiredPos['y'] - robot_pos['y'])**2) < 0.02):
            DesiredPos = {'x':0.5,"y":0}
            ball_angle, robot_angle = get_angles(DesiredPos, robot_pos)
            return goTo(DesiredPos["x"], DesiredPos["y"], robot_pos, robot_angle, point_goal=True, ignor=False)
    
    AngleNDistance = GetAngleToSpot(x,y,robot_pos, robot_angle) #0 angle to spot, 1 distance to spot
    
    MotorsSpeed = RotateToSpot(AngleNDistance[0], magicnum, AngleNDistance[1], should_soften, point_goal) #0 right, 1 left 
    
    return [MotorsSpeed[0], MotorsSpeed[1]]

##################################################
##################################################
def GetAngleToSpot(x,y,robot_pos, robot_angle, target_angle=-1):
    YAxisToDest = (1-y)-(1-robot_pos["y"])
    XAxisToDest = x-robot_pos["x"]
    angle = math.atan2(YAxisToDest, XAxisToDest)
    #gets the hypo to know the distance between robot and desired location
    distanceToSpot = math.sqrt(math.pow(YAxisToDest, 2) + math.pow(XAxisToDest, 2))                   
    #print(angle) 
    #some math (copied xD) to convert the value we got into degrees so that our robot can get the direction to point at  
    if angle < 0:
        angle = 2 * math.pi + angle

    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    robotDestAngle = math.degrees(angle + robot_angle)
    robotDestAngle -= 90
    if robotDestAngle > 360:
        robotDestAngle -= 360
    #print("angle: ",  robotDestAngle, " distance: " , distanceToSpot )
    
    return [robotDestAngle, distanceToSpot]

#####################################################
#####################################################
def RotateToSpot (robotAngleFromSpot, magicnum, dst, should_soften, point_goal):
    #defualt values
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    MagicNum = magicnum
    #67
    
    angleToTarget = 360-robotAngleFromSpot #actual rotation needed to face desires spot
    #print (angleToTarget)
    if(angleToTarget > 360): # for some reason the fieled is 360...its 400 somethin i think
        angleToTarget-=360
    right = -10 if not point_goal else 0
    left = -10 if not point_goal else 0
    #64-
    if((angleToTarget > 0 and angleToTarget <90) or (angleToTarget > 270 and angleToTarget <360)):
        if(angleToTarget >=0 and angleToTarget <180):#turn left
            left += (angleToTarget *MagicNum)
            right  +=(-angleToTarget)*MagicNum
        if(angleToTarget <= 360 and angleToTarget>=180):#turn right  
            angleToTarget = 360 - angleToTarget
            left += (-angleToTarget)*MagicNum 
            right +=  (angleToTarget*MagicNum)#*2
    else:
        right = 10 if not point_goal else 0
        left = 10 if not point_goal else 0
        angleToTarget -= 180
        if angleToTarget < 0:
            angleToTarget+=360
        if(angleToTarget >=0 and angleToTarget <180):#turn left
            
            left -= (-angleToTarget *MagicNum)
            right  -=(angleToTarget)*MagicNum
        if(angleToTarget <= 360 and angleToTarget>=180):#turn right  
            angleToTarget = 360 - angleToTarget
            left -= (angleToTarget)*MagicNum 
            right -=  (-angleToTarget*MagicNum)#*2

    if dst < 0.1 and should_soften:
        right = right*dst*23
        left = left*dst*23
    # if motors speed exceed the limit. cap em
    if(right >10):
        right= 10
    elif(right <-10):
        right= -10

    if(left >10):
        left= 10
    elif(left <-10):
        left= -10
    


    return [right, left]
