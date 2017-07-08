import rospy

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap


def generate_trajectory(vehicle_state, obstacle_map):
    trajectory = Trajectory()
    rospy.loginfo("generate trajectory!")
    return trajectory