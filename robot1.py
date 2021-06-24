
# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math

# You can also import scripts that you put into the folder with controller
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils

from intercepts import interceptCalculator
from CoordinateRecalculator import coor_recalc, robot_pos_recalc, inverse_point
from GoToFunc import goTo
from MovementCalculator import fit_parabola, get_tangent_point, passes_boundary, scores_own_goal

STARTING_POS =  {'x': 0.21979533333333334, 'y': 0.7072653846153846}
TARGET_POS_Y1 = {'x': STARTING_POS['x'], 'y': 0}
CAMPING_POS_Y1 = {'x': STARTING_POS['x'], 'y':1}

def BALL_IN_QUADRANT_Y1(ball):
    if ball['x']< 0.36 and ball['y']> 0.57:
        return True
    return False


class MyBallPassingRobot1(RCJSoccerRobot):
    def run(self):
        self.intercept_c = interceptCalculator(3)
        while self.robot.step(TIME_STEP) != -1:
            Team = (self.team == "B")

            if self.is_new_data():
                data = self.get_new_data()
                #due to extensive openAI gym testing we know that desync DOES occur
                while self.is_new_data():            
                    data = self.get_new_data()

                # Get the position of our robot
                robot_pos = robot_pos_recalc(data[self.name], Team)
                # Get the position of the ball
                ball_pos = coor_recalc(data['ball']['x'], data['ball']['y'], Team)
                print(ball_pos)

                self.intercept_c.pushPoint(ball_pos)
                
                if BALL_IN_QUADRANT_Y1(ball_pos):
                    intercept = self.intercept_c.calculateOptimumIntercept(robot_pos, Team, sample_count=200)
                    
                    
                    inverse_intercept = inverse_point(intercept)
                    inverse_robot_pos = inverse_point(robot_pos)
                    
                    inverse_target = inverse_point(TARGET_POS_Y1)
                    #dst
                    print(inverse_robot_pos)
                    print(inverse_intercept)
                    print(inverse_target)


                    x = fit_parabola(inverse_intercept, inverse_robot_pos , inverse_target)

                    x['a'] = (1/0.96)*x['a']
                    point = get_tangent_point(inverse_robot_pos, x, Team,is_inverse=True)
                    ball_angle, robot_angle = self.get_angles(point, robot_pos)
                    print("point", point)
                    out = goTo(point['y'], point['x'], robot_pos, robot_angle, should_soften=False)
                else:
                    all_angle, robot_angle = self.get_angles(CAMPING_POS_Y1, robot_pos)
                    out = goTo(CAMPING_POS_Y1['x'], CAMPING_POS_Y1['y'], robot_pos, robot_angle, should_soften=False)
                # Get angle between the robot and the ball
                # and between the robot and the north

                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])

