import rospy
import time
import ConfigParser

import vehicleControl

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

cf = ConfigParser.ConfigParser()
cf.read("../config/planning.conf")
init_pos = cf.get("init", "init_pos")
init_speed = cf.get("init", "init_speed")
init_route = cf.get("init", "init_route")


def callback_trajectory():
    print "callback traj:", traj

def init_sim():
    vehicleControl.init()
    vehicleControl.init_vehicle(init_pos, init_speed, init_route)

if __name__=="__main__":
    init_sim()

    rospy.init_node("sumo")
    rospy.Subscriber("/planning/trajectory", Trajectory, callback_trajectory)
    pub_localize = rospy.Publisher("/simulation/localize", Pose, queue_size=10)
    pub_obstacle = rospy.Publisher("/simulation/obstacles", ObstacleMap, queue_size = 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vehicleControl.do_step()
        localize = vehicleControl.get_localize()
        obs_map = vehicleControl.get_obstacles()
        pub_localize.publish(localize)
        pub_obstacle.publish(obs_map)
        rate.sleep()

    vehicleControl.destroy()