from collections import deque
import time
class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []
        original_pos = self.planning_env.robot.GetTransform()
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        # Peter's implement
        #create a piority queue and inital with start_config's node_id
        start_time = time.time()

        if( self.visualize and hasattr(self.planning_env, 'InitializePlot')):
            self.planning_env.InitializePlot(goal_config)


        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        pio_q = [start_id]; # for BFS use pio_q = deq([start_id])
        
        total_vertices = 0;

        #insert the visited node in slef.nodes
        self.nodes[start_id] = start_id
        find_path = False
        total_vertices += 1
        
        while(len(pio_q)):
            #raw_input("push any key to continue")
            #print "pio_q = ", pio_q
            pop_id = pio_q.pop();# for BFS change to popleft()
            #print "popout = ", pop_id
            if(pop_id == goal_id):
                #visualize
                if(self.visualize):
                    pop_config = self.planning_env.discrete_env.NodeIdToConfiguration(pop_id)
                    self.planning_env.PlotEdge(pop_config,goal_config)
                find_path = True
                break 
            neighbors = self.planning_env.GetSuccessors(pop_id);
            #print "neighbor = ",neighbors
            for n_id in neighbors:
                #create 
                n_config = self.planning_env.discrete_env.NodeIdToConfiguration(n_id)
                #check no visited before        and  can acheive this neighbor(no collision)
                if( (n_id in self.nodes) == False):
                    #insert in priority queue and mark as visisted
                    pio_q.append(n_id)
                    #mark as visited and point to parent id
                    self.nodes[n_id] = pop_id
                    total_vertices += 1
                    #for visulization
                    if(self.visualize):
                        pop_config = self.planning_env.discrete_env.NodeIdToConfiguration(pop_id)
                        self.planning_env.PlotEdge(pop_config,n_config)
                        #print n_config
        
        #find a path
        self.planning_env.robot.SetTransform(original_pos)
        if(find_path):
            current_id = goal_id
            #recursively search from goal to start
            while(self.nodes[current_id] != start_id):
                current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current_id)
                plan.append(current_config)
                current_id = self.nodes[current_id]
            #add start_config
            plan[0] = goal_config
            plan.append(start_config)
        else:
            print "No find path"
            return plan
        #reverse the path
        plan = plan[::-1]
        #print for time, path length and total tree vertices
        total_time = time.time() - start_time
        print "total plan time = ",total_time
        
        print "total path length",self.planning_env.ComputePathLength(plan)

        print "total visited vertices", total_vertices

        return plan

