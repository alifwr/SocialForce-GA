import cv2
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class Visualizer():
    def __init__(self) -> None:
        self.staticReady = False
        self.dynamicReady = False
        self.goalReady = False
        self.resultantReady = False
        self.screenHeight = 480
        self.screenWidth = 640
        self.staticPos = Point()
        self.dynamicPos = Point()
        self.goalPos = Point()
        self.resultantPos = Point()
        rospy.init_node('visualizer', anonymous=True)
        rospy.Subscriber('/static_object', Point, self.updateStatic)
        rospy.Subscriber('/dynamic_object', Point, self.updateDynamic)
        rospy.Subscriber('/goal_object', Point, self.updateGoal)
        rospy.Subscriber('/hokuyo', LaserScan, self.updateLaser)
        rospy.Subscriber('/sfm_result', Point, self.updateResultant)

        timer = rospy.Timer(rospy.Duration(0.2), self.draw)

        rospy.spin()
        timer.shutdown()
    
    def draw(self, event):
        if(not rospy.is_shutdown()):
            white = (255, 255, 255)
            blue = (255, 0, 0)
            green = (0, 255, 0)
            red = (0, 0, 255)

            scale = self.screenHeight/5
            self.screen = np.zeros((self.screenHeight, self.screenWidth,3), np.uint8)
            center = (int(self.screenWidth/2), int(self.screenHeight*2/3))
            cv2.circle(self.screen, center, int(self.screenHeight/30), white, 1)

            if(self.staticReady):
                staticObject = (int((self.staticPos.x*scale)+center[0]), int((-self.staticPos.y*scale)+(center[1])))
                cv2.line(self.screen, center, staticObject, red, 1)
            if(self.dynamicReady):
                dynamicObject = (int((self.dynamicPos.x*scale)+center[0]), int((-self.dynamicPos.y*scale)+center[1]))
                cv2.line(self.screen, center, dynamicObject, blue, 1)
            if(self.goalReady):
                goalObject = (int((self.goalPos.x*scale)+center[0]), int((-self.goalPos.y*scale)+center[1]))
                cv2.line(self.screen, center, goalObject, green, 1)
            if(self.resultantReady):
                resultantObject = (int((self.resultantPos.x*scale)+center[0]), int((-self.resultantPos.y*scale)+center[1]))
                cv2.line(self.screen, center, resultantObject, green, 2)

            for index, range in enumerate(self.ranges):
                if(range<5):
                    # index = self.ranges.index(range)
                    n_data = len(self.ranges)
                    angle = (240 * (float(index) / float(n_data))) - 30
                    radian = math.radians(angle)
                    x = int(math.cos(radian) * range * scale) + center[0]
                    y = int(-math.sin(radian) * range * scale) + center[1]
                    cv2.circle(self.screen, (x,y), 1, green, 1)
            
            cv2.imshow('screen', self.screen)
            cv2.waitKey(1)
    
    def updateLaser(self, data):
        self.ranges = data.ranges

    def updateStatic(self, data):
        self.staticPos = data
        self.staticReady = True
    
    def updateDynamic(self, data):
        self.dynamicPos = data
        self.dynamicReady = True
    
    def updateGoal(self, data):
        self.goalPos = data
        self.goalReady = True
    
    def updateResultant(self, data):
        self.resultantPos = data
        if(self.resultantPos.x != self.resultantPos.x):
            self.resultantPos.x = 0
        if(self.resultantPos.y != self.resultantPos.y):
            self.resultantPos.y = 0
        self.resultantReady = True

if __name__ == '__main__':
    try:
        visualizer = Visualizer()
    except rospy.ROSInterruptException:
        pass