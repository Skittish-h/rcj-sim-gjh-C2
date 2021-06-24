
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
TARGET_POS_Y2 = {'x': 1, 'y': 1-STARTING_POS['y']}
CAMPING_POS_Y2 = {'x': 0, 'y':TARGET_POS_Y2['y']}

def BALL_IN_QUADRANT_Y2(ball):
    if ball['x'] < 0.36 and ball['y']<  0.41:
        return True
    return False


class MyBallPassingRobot2(RCJSoccerRobot):
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
                
                if BALL_IN_QUADRANT_Y2(ball_pos):
                    intercept = self.intercept_c.calculateOptimumIntercept(robot_pos, Team, sample_count=200)
                    


                    x = fit_parabola(intercept, robot_pos , intercept)

                    x['a'] = (0.96)*x['a']
                    point = get_tangent_point(robot_pos, x, Team)
                    ball_angle, robot_angle = self.get_angles(point, robot_pos)
                    print("point", point)
                    out = goTo(point['x'], point['y'], robot_pos, robot_angle, should_soften=False)
                else:
                    all_angle, robot_angle = self.get_angles(CAMPING_POS_Y2, robot_pos)
                    out = goTo(CAMPING_POS_Y2['x'], CAMPING_POS_Y2['y'], robot_pos, robot_angle, should_soften=False)
                # Get angle between the robot and the ball
                # and between the robot and the north

                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])

