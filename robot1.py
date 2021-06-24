
# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math

# You can also import scripts that you put into the folder with controller
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils

from intercepts import interceptCalculator
from CoordinateRecalculator import coor_recalc, robot_pos_recalc
from GoToFunc import goTo
from MovementCalculator import fit_parabola, get_tangent_point, passes_boundary, scores_own_goal


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
                
                self.intercept_c.pushPoint(ball_pos)
                
                intercept = self.intercept_c.calculateOptimumIntercept(robot_pos, Team, sample_count=200)

                x = fit_parabola(intercept, robot_pos ,{'x':(0.0 if Team else 1.0),"y":0.5})
                if not passes_boundary(x):
                    point = get_tangent_point(robot_pos, x, Team)
                    ball_angle, robot_angle = self.get_angles(point, robot_pos)
                
                    out = goTo(point['x'], point['y'], robot_pos, robot_angle, should_soften=False)
                # Get angle between the robot and the ball
                # and between the robot and the north

                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])

