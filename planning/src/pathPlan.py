import rospy

from rrt.rrtStar import Planning
from rrt.visualization import print_map

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

localize = Pose()
obstacle_map = ObstacleMap()
localize_ready = False
obstacle_ready = False

def callback_localize(msg):
    localize = msg
    global localize_ready
    localize_ready = True
    # rospy.loginfo("localize callback.")

def callback_obstacle(msg):
    obstacle_map = msg
    global obstacle_ready
    obstacle_ready = True
    # rospy.loginfo("obstacle callback.")

if __name__=="__main__":
    rospy.init_node("path_plan")
    rospy.Subscriber("/simulation/localize", Pose, callback_localize)
    rospy.Subscriber("/simulation/obstacles", ObstacleMap, callback_obstacle)
    pub_trajectory = rospy.Publisher("/planning/trajectory", Trajectory, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if localize_ready and obstacle_ready:
            print "localize ready."
            print_map(localize, obstacle_map)
            plan = Planning(localize, obstacle_map)
            trajectory = plan.generate_trajectory()
            print "Generated trajectory!"
            break
            # rrtStar.generate_trajectory(localize, obstacle_map)
        rate.sleep()
