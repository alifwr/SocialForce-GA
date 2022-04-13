import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class Scanner:
    def __init__(self) -> None:
        rospy.init_node('object_detector', anonymous=True)
        rospy.Subscriber('/hokuyo', LaserScan, self.callback)
        
        self.staticPub = rospy.Publisher('/static_object', Point, queue_size=10)
        self.dynamicPub = rospy.Publisher('/dynamic_object', Point, queue_size=10)
        
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
        static_msg = {'x':x, 'y':y, 'z':0}
        static_point = [x, y, 0]
        static_point = Point(x, y, 0)
        self.staticPub.publish(static_point)

if __name__ == '__main__':
    try:
        scanner = Scanner()
    except rospy.ROSInterruptException:
        pass
