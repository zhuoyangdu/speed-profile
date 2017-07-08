import rospy

import rrtStar

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

localize = Pose()
obstacle_map = ObstacleMap()

def callback_localize(msg):
    localize = msg
    rospy.loginfo("localize callback.")

def callback_obstacle(msg):
    obstacle_map = msg
    rospy.loginfo("obstacle callback.")

if __name__=="__main__":
    rospy.init_node("path_plan")
    rospy.Subscriber("/simulation/localize", Pose, callback_localize)
    rospy.Subscriber("/simulation/obstacles", ObstacleMap, callback_obstacle)
    pub_trajectory = rospy.Publisher("/planning/trajectory", Trajectory, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rrtStar.generate_trajectory(localize, obstacle_map)
        rate.sleep()
