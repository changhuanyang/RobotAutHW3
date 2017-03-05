from Queue import PriorityQueue


class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []

        original_pos = self.planning_env.robot.GetTransform()

        if( self.visualize and hasattr(self.planning_env, 'InitializePlot')):
            self.planning_env.InitializePlot(goal_config)

        a_map = PriorityQueue()
        trajectory = {}
        overall_cost = {}

        start = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        end = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        trajectory[start] = None
        overall_cost[start] = 0
        path_found = False

        a_map.put(start,0)

        while not a_map.empty():
            current_node = a_map.get()
        
            if current_node == end:
                path_found = True
                break
        
            for neighbour in self.planning_env.GetSuccessors(current_node):

                if self.planning_env.is_collision(self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)) == False:
                    new_cost = overall_cost[current_node] + self.planning_env.ComputeDistance(current_node, neighbour)
                    if (neighbour not in overall_cost or new_cost < overall_cost[neighbour]) :
                        overall_cost[neighbour] = new_cost
                        priority_cost = new_cost + self.planning_env.ComputeHeuristicCost(end, neighbour)
                        a_map.put(neighbour, priority_cost)
                        trajectory[neighbour] = current_node
                        # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_node),self.planning_env.discrete_env.NodeIdToConfiguration(neighbour))
                        # print "cost: " cost_so_far[next],"came from: "came_from[next] 

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        self.planning_env.robot.SetTransform(original_pos)
                
        current_id = end
        if path_found :
            print "Path found"
            while(trajectory[current_id] != start):
                plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current_id))
                current_id = trajectory[current_id]
        else:
            print "No path found"
        
        plan[0] = goal_config
        plan.append(start_config)
        plan = plan[::-1]

        return plan
