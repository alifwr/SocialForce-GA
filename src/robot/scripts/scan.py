import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D

class Scanner:
    def __init__(self) -> None:
        rospy.init_node('object_detector', anonymous=True)
        rospy.Subscriber('/hokuyo', LaserScan, self.callback)
        rospy.Subscriber('/robot_pose', Pose2D, self.updateRobot)
        rospy.Subscriber('/bill_pos', Point, self.updateBill)
        rospy.Subscriber('/target_pos', Point, self.updateTarget)
        
        self.ranges = ()
        self.robotPose = Pose2D()
        self.billPos = Point()
        self.targetPos = Point()

        self.staticPub = rospy.Publisher('/static_object', Point, queue_size=10)
        self.dynamicPub = rospy.Publisher('/dynamic_object', Point, queue_size=10)
        self.targetPub = rospy.Publisher('/goal_object', Point, queue_size=10)
        
        # rate = rospy.Rate(10) # 10hz
        rospy.spin()
        #pass

    def callback(self, data):
        self.ranges = data.ranges
        self.n_data = len(self.ranges)

        self.nearestStaticRange = min(self.ranges)
        self.nearestStaticIndex = self.ranges.index(self.nearestStaticRange)
        self.nearestStaticAngle = (240 * (float(self.nearestStaticIndex) / float(self.n_data))) - 30
        x = math.cos(math.radians(self.nearestStaticAngle)) * self.nearestStaticRange
        y = math.sin(math.radians(self.nearestStaticAngle)) * self.nearestStaticRange
        static_point = Point(x, y, 0)
        dynamic_point = Point(self.billPos.x - self.robotPose.x, self.billPos.y - self.robotPose.y, 0)
        dynamic_point = self.frameRotation(Pose2D(0,0, self.robotPose.theta), dynamic_point)
        target_point = Point(self.targetPos.x - self.robotPose.x, self.targetPos.y - self.robotPose.y, 0)
        target_point = self.frameRotation(Pose2D(0,0, self.robotPose.theta), target_point)
        self.staticPub.publish(static_point)
        self.dynamicPub.publish(dynamic_point)
        self.targetPub.publish(target_point)
    
    def frameRotation(self, robot, object):
        # print(robot)
        a = robot.x
        b = robot.y
        theta = robot.theta
        x = object.x
        y = object.y
        # if(x!=0 and y!=0):
        #     theta_ = math.degrees(math.atan(y/x))
        # else:
        #     theta_ = 0
        rotation = (90 - theta)
        rotRad = math.radians(rotation)
        x_ = (((x - a) * math.cos(rotRad)) - ((y - b) * math.sin(rotRad)) + a)
        y_ = (((x - a) * math.sin(rotRad)) + ((y - b) * math.cos(rotRad)) + b)
        return Point(x_, y_, 0)

    def updateRobot(self, data):
        self.robotPose = data
    
    def updateBill(self, data):
        self.billPos = data
    
    def updateTarget(self, data):
        self.targetPos = data

if __name__ == '__main__':
    try:
        scanner = Scanner()
    except rospy.ROSInterruptException:
        pass
