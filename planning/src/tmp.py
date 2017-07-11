class Node:
    def __init__():
        self.time
        self.distance
        self.velocity
        self.parent_id
        self.self_id
        self.cost

    def set_parent_node():
    def get_parent_node():
    def set_cost():

class Tree:
    def __init__():
        self.nodes

    def tree_size():

    def nearest():

    def near_lower_region():

    def near_upper_region():

    def steer():

    def vertex_feasible():
        return kinematic_feasible() & collision_free()

    def estimate_velocity():

    def estimate_acceleration():

    def rebuild_tree():

    def get_parent_path():

    def extend_tree(sample):

class Obstacles():
    def __init__():
        self.obstacle_map = ObstacleMap()

    def collision_free():

    def risk_assessment():

class Cost
    def get_node_cost():

    def get_path_cost():

def planner():
    # init
    tree = Tree()
    while:
        sample = sample(t,s)
        # Extend
        tree.extend_tree(sample)
        # ReachingGoal
        if reaching_goal(new_node):
            path = get_parent_path()
            path_cost = get_path_cost()
            print path

