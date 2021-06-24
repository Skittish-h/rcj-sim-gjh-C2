from statistics import mean
import math
from MovementCalculator import fit_parabola, passes_boundary
#                GJH_team                 #
##########Intercept Calc Class#############
# Class processes past points of ball to  #
# find optimal place for the interception #
# of the ball.                            #
###########################################


class interceptCalculator():
    
    #constructor
    #sample depth - amount of samples remembered to estimate ball trajectory; >1
    #default - intial ball position
    def __init__(self,sample_depth, default = {"x":0.5,"y":0.5}):
        self.sample_depth = sample_depth
        self.pastIntercepts = [default for i in range(sample_depth)]
    
    #print's balls point history
    def printPointHistory(self):
        print(self.pastIntercepts)
    

    #estimate gradient of
    # returns m in function future_cord = current + m*time
    def estimateFunction(self, coordinate): 
        #m & c values we averge out
        m_vals = []

        #for every calculatable interval (since 0th term is last and each interval is calcaulated by (new-old)/time)
        for i in range(self.sample_depth-1, -1,-1):
            for b in range(i-1,-1,-1):
                #calculate m: dDistance/dTime
                
                m = (self.pastIntercepts[i][coordinate] - self.pastIntercepts[b][coordinate])/(i-b)
                #calculate c: c = dDistance - m*dTime 
                m_vals.append(m)
        
        return mean(m_vals)
    def pushPoint(self, point):
        self.pastIntercepts.pop(0)
        self.pastIntercepts.append(point)
    
    #calculates time nessecary to traverse a certain distance - used to calculate imtercept
    def calculate_time(self, distance):
        speed_constant = {'m':110, 'c':0}
        
        return speed_constant['m']*distance + speed_constant['c']
        
    #function figuring if the ball is going to hit the robot
    #done by estimating the X time for ball to travel to robot, and thus getting the Y coordinate offset
    
    def should_kick(self, currentPositioning, functions):
        #some data collection r-> robot pos; b-> ball pos
        r = currentPositioning
        b = self.pastIntercepts[self.sample_depth-1]

        #getting distance between ball and robot
        X_dst =  r['x'] - b['x']
        Y_dst =  r['y'] - b['y'] 
        
        #feeding this data into our functions to see if they correlate, can't have zero division
        if functions['x'] != 0:
            #calculate time to X
            time_to_X = X_dst/functions['x']
            error = 1000
            if time_to_X > 0:
            #error is deviation from funtioned Y movenet and X duration
                error = Y_dst - functions['y']*time_to_X
            
            #give error, direction of ball travel, and time to intersection of ball and robot
            return {"err": error, "dir":"B" if functions['x']>0 else "Y", "time":time_to_X, "x":X_dst, 'xm':functions['x']}
        return {"err": 100, "dir":"B" if functions['x']>0 else "Y", "time":0, "x":X_dst,'xm':functions['x']}
        
    def get_desired_hit_angle(self, m_vals, intercept, desired_target={'x':0, 'y':0.5}):
        a_to_target = self.get_angles(desired_target, intercept)
        ball_vector_a = self.get_angles(m_vals,{'x':0, "y":0})
        
        adjustment = (a_to_target-ball_vector_a)
        return a_to_target, ball_vector_a
    #function that calculates the optimum intercept 
    #really long but most of it is just renaming variables and arguments for better visibility and explenations
    def calculateOptimumIntercept(self, currentPositioning, team ,sample_count=50,sample_accuracy=1):
        #very complex calculations incoming
        #all distances we return
        distances = []
        
        #previous X and Y coordinates for colision calc
        prev_b_X = 0
        prev_b_Y = 0

        #time offsets for when collision occurs
        t_offset= 0
        #b: ball x & y
        b = self.pastIntercepts[self.sample_depth-1]
        #r: robot x & y
        r = currentPositioning
        #m: gradients
        m = {'x': self.estimateFunction('x'), 'y':self.estimateFunction('y')}
        initial_mx = {'x': self.estimateFunction('x'), 'y':self.estimateFunction('y')}
        #data for decision to kick
        #do sample_count loops with sample accuracy time each
        for t in range(0, sample_count*sample_accuracy, sample_accuracy):
            #t1 = time elapsed since beginning/last collision
            t1 = (t - t_offset)
            
            #calculated future BallX & BallY
            ballx = b['x'] + (t1 * m['x'])
            bally = b['y'] + (t1 * m['y'])
            

            ## *Colision Checks* ##
            
            #the way we compute ricochet's:
            #   -in a prediction, check if balls position isn't negative or > 1in direction
            #   -if it is:
            #       -we asume that the last "OK" coordinate is the riccochet point
            #       -we assume collision is elastic (all k_energy is preserved) (TODO: we might want to calculate collision elasticity)
            #       -offset time to future predictions by the current t ("t_offset" variables)
            #       -invert gradient of travel("m") constant
            #       -set past ball position ("b") to previous X & Y


            #ball is to pass X boundary
            if(ballx < 0 or ballx > 1):
                t_offset = t
                b = {'x':prev_b_X,'y':prev_b_Y}
                m['x'] = -m['x']

                
            if(bally < 0 or bally > 1):
                t_offset = t
                b = {'x':prev_b_X,'y':prev_b_Y}
                m['y'] = -m['y']

            #ball is to pass X boundary
            #print(ballx, bally)
            
            #do math
            distance_from = math.sqrt(((ballx - r['x'])**2) + ((bally - r['y'])**int(2)))
            
            #if we can travel to the ball in time by that coordinate, return
            raw_time = self.calculate_time(distance_from)
            if (m['x']>0 and ballx > r['x']) if team else (m['x']<0 and ballx < r['x']):
                raw_time+=10
            else:
                raw_time += abs((ballx-r['x']))
                #print("ahead")
            if(raw_time <= t):
                return {"isIntercept":True, "x":ballx, "y":bally, "t":t, "kik": False}#abs(self.should_kick(r, initial_mx)['err'])<0.05}
            

            #sacrifice memory for processing
            prev_b_X = ballx
            prev_b_Y = bally

        return {"isIntercept":False, "x":0, "y":0, "t":1000,"kik":False}
    
    #function that calculates target position in case of an own goal possible scenario.
    def normalize_vec(self, vec):
        divisor = math.sqrt(vec[0]**2 + vec[1]**2)
        #null division actually happened here
        const = 1/(divisor if divisor != 0 else 0.0001)
        return [vec[0]*const, vec[1]*const]

    def multiply_vec(self, const, vec):
        return [const*vec[0], const*vec[1]]
    
    def add_vec_to_point(self, vec, point):
        return {'x': point['x'] + vec[0], 'y' : point['y'] + vec[1]}

    def ball_doge_pos(self, ball_pos, robot_pos):
        #spaghetti code:
        
        print("dodging")
        #strategy: find ball vector 
        #   -> find the two perpendicular vectors, and then turn them into unit vectors
        #   -> multiply by constant
        #   -> 

        #ball vector
        ball_vector_x = self.estimateFunction('x')
        ball_vector_y = self.estimateFunction('y')

        #two potential vectors - normalized
        left_vector = self.normalize_vec([-ball_vector_y, ball_vector_x])
        right_vector = self.normalize_vec([ball_vector_y, -ball_vector_x])
        
        #multiplied by magical scalar
        MAGIC_CONST = 0.04
        left_vector =  self.multiply_vec(MAGIC_CONST, left_vector)
        right_vector = self.multiply_vec(MAGIC_CONST, right_vector)

        #two potential points
        left_potential_point = self.add_vec_to_point(left_vector, ball_pos)
        right_potential_point = self.add_vec_to_point(right_vector, ball_pos)

        #distance from robot to each potential point w/ pythagoras
        left_dst = math.sqrt((left_potential_point['x'] - robot_pos['x'])**2 + (left_potential_point['y'] - robot_pos['y'])**2)
        right_dst =math.sqrt((right_potential_point['x'] - robot_pos['x'])**2 + (right_potential_point['y'] - robot_pos['y'])**2)

        #return closest point
        # if(left_dst > right_dst):
        #     return right_potential_point
        # return left_potential_point
        if(left_dst > right_dst):
            return self.add_vec_to_point(right_vector, robot_pos)
        return self.add_vec_to_point(left_vector, robot_pos)  