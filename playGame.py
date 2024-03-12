#!/usr/bin/env python3

import rospy,math
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import ticMiniMax

#Class to go to a given point on a map
class GoToPose():
    def __init__(self, robNum):

        self.goal_sent = False

	    # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
    	# Tell the action client that we want to spin a thread by default
        mbString = "/robot_" + str(robNum) + "/move_base"
        self.move_base = actionlib.SimpleActionClient(mbString, MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

	    # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    #Go to a specific point
    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class Robot:

    def __init__(self, game_piece, idle_spot, number):
        self.turtle_pose = Pose()
        self.board_state = [['_', '_', '_'],
                            ['_', '_', '_'],
                            ['_', '_', '_']]
        self.navigator = GoToPose(number)
        self.game_piece = game_piece
        self.idle_spot = idle_spot
        self.number = number
    
    def isGameOver(self):
        # Checking for Rows for X or O victory.  
        for row in range(3) :      
            if (self.board_state[row][0] == self.board_state[row][1] and self.board_state[row][1] == self.board_state[row][2]) :         
                return True
    
        # Checking for Columns for X or O victory.  
        for col in range(3) : 
            if (self.board_state[0][col] == self.board_state[1][col] and self.board_state[1][col] == self.board_state[2][col]) : 
                return True
    
        # Checking for Diagonals for X or O victory.  
        if (self.board_state[0][0] == self.board_state[1][1] and self.board_state[1][1] == self.board_state[2][2]) : 
            return True
    
        if (self.board_state[0][2] == self.board_state[1][1] and self.board_state[1][1] == self.board_state[2][0]) : 
            return True
        
        return False
    
    def won(self):
        #Checking for a win in the rows
        for row in range(3) :      
            if (self.board_state[row][0] == self.board_state[row][1] and self.board_state[row][1] == self.board_state[row][2]) :         
                if (self.board_state[row][0] == self.game_piece) : 
                    return True
                else:
                    return False
    
        #Checking for a win in the columns
        for col in range(3) : 
            if (self.board_state[0][col] == self.board_state[1][col] and self.board_state[1][col] == self.board_state[2][col]) : 
                if (self.board_state[0][col] == self.game_piece) :  
                    return True
                else:
                    return False
    
        #Checking for a win in the diagonals
        if (self.board_state[0][0] == self.board_state[1][1] and self.board_state[1][1] == self.board_state[2][2]) : 
            if (self.board_state[0][0] == self.game_piece) : 
                return True
            else:
                return False
    
        if (self.board_state[0][2] == self.board_state[1][1] and self.board_state[1][1] == self.board_state[2][0]) : 
            if (self.board_state[0][2] == self.game_piece) : 
                return True
            else:
                return False
    
    #Spin 360 degrees to indicate placing a piece
    def makeMove(self):
        self.rotate(degrees2radians(359.9), True, 30)

    #Win routine, spin slightly quicker 3 times
    def celebrate(self):
        self.rotate(degrees2radians(359.9), True, 35)
        self.rotate(degrees2radians(359.9), True, 35)
        self.rotate(degrees2radians(359.9), True, 35)
    
    #Loss routine, "shake head" back and forth slowly
    def beSad(self):
        self.rotate(degrees2radians(90.0), False, 20)
        self.rotate(degrees2radians(180.0), True, 20)
        self.rotate(degrees2radians(90.0), False, 20)
    
    #Neutral tie routine, spin once at a normal speed
    def tie(self):
        self.rotate(degrees2radians(359.9), True, 30)
        
    #Helper function to rotate robot
    def rotate(self, relative_angle, isClockwise, speed):
        pubString = "/robot_" + str(self.number) + "/cmd_vel"
        pub = rospy.Publisher(pubString, Twist, queue_size = 10)

        outData = Twist()

        t0 = rospy.get_rostime().secs
        while t0 == 0:
            t0 = rospy.get_rostime().secs

        current_angle = 0

        rate = rospy.Rate(10)

        if isClockwise:
            outData.angular.z = degrees2radians(speed * -1)
        else:
            outData.angular.z = degrees2radians(speed)

        while current_angle < relative_angle:
            pub.publish(outData)
            t1 = rospy.get_rostime().secs
            while t1 == 0:
                t1 = rospy.get_rostime().secs
            current_angle = degrees2radians(30) * (t1 - t0)
            rate.sleep()
    
    #Add piece to board
    def updateBoard(self, newMove, gamePiece):
        self.board_state[newMove[0]][newMove[1]] = gamePiece

def degrees2radians(angle):
    return angle * (math.pi/180.0)


#Converting a move on the virtual board to the real life position
def moveToPose(move):
    if move == (0, 0):
        return {'x': 1.000, 'y' : 0.500}
    elif move == (0, 1):
        return {'x': 1.000, 'y' : 0.000}
    elif move == (0, 2):
        return {'x': 1.000, 'y' : -0.500}
    elif move == (1, 0):
        return {'x': 0.667, 'y' : 0.500}
    elif move == (1, 1):
        return {'x': 0.667, 'y' : 0.000}
    elif move == (1, 2):
        return {'x': 0.667, 'y' : -0.500}
    elif move == (2, 0):
        return {'x': 0.333, 'y' : 0.500}
    elif move == (2, 1):
        return {'x': 0.333, 'y' : 0.000}
    elif move == (2, 2):
        return {'x': 0.333, 'y' : -0.500}
    

        

if __name__ == '__main__':
    try:
        rospy.init_node('tictactoe', anonymous=True)
        miniMax = ticMiniMax.MiniMax()

        rob1 = Robot('X', {'x': 0.000, 'y' : 0.000}, 1)
        rob2 = Robot('O', {'x': 0.000, 'y' : -0.500}, 2)
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}


        curRobot = rob1
        idleRobot = rob2

        while not curRobot.isGameOver():
            nextMove = miniMax.findBestMove(curRobot.board_state)
            nextPose = moveToPose(nextMove)
            curRobot.navigator.goto(nextPose, quaternion)
            curRobot.makeMove()
            curRobot.updateBoard(nextMove, curRobot.game_piece)
            idleRobot.updateBoard(nextMove, curRobot.game_piece)
            curRobot.navigator.goto(curRobot.idle_spot, quaternion)
            curRobot, idleRobot = idleRobot, curRobot
        

        if rob1.won():
            rob1.celebrate()
            rob2.beSad()
        elif rob2.won():
            rob1.beSad()
            rob2.celebrate()
        else:
            rob1.tie()
            rob2.tie()


        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


