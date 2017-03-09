import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.boundary_limits = [[-5., -5.], [5., 5.]]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))
        #print config
        idx = 0
        for i in range(len(coord)):
            if coord[i] + 1 < self.discrete_env.num_cells[i]:
                this_coord = numpy.copy(coord)
                this_coord[i] += 1
                this_id = self.discrete_env.GridCoordToNodeId(this_coord)
                this_config = self.discrete_env.NodeIdToConfiguration(this_id)
                if self.no_collision(this_config):
                    successors.append(this_id)
            if (coord[i] - 1) >= 0 :
                this_coord = numpy.copy(coord)
                this_coord[i] -= 1
                this_id = self.discrete_env.GridCoordToNodeId(this_coord)
                this_config = self.discrete_env.NodeIdToConfiguration(this_id)
                if self.no_collision(this_config):
                    successors.append(this_id)
        #print "number of successors = ",len(successors)
        return successors

    def ComputeDistance(self, start_id, end_id):
        dist = numpy.linalg.norm(numpy.array(self.discrete_env.NodeIdToConfiguration(start_id)) - numpy.array(self.discrete_env.NodeIdToConfiguration(end_id)))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by te two node ids
        return dist
    def ComputeDistance_continue(self, start_id, end_id):
        dist = numpy.linalg.norm(numpy.array(start_id) - numpy.array(end_id))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by te two node ids
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        # cost = self.ComputeDistance(start_id,goal_id)
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)

        cost = 0;
        for i in range(self.discrete_env.dimension):
            cost = cost + abs(start_config[i] - goal_config[i])

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
    #help function for checking collision
    def no_collision(self, n_config):
        with self.robot:
            position = self.robot.GetTransform();
        
            #change the current position valuse wiht n_config
            position[0][3] = n_config[0]
            position[1][3] = n_config[1]
            #move robot to new position
            self.robot.SetTransform(position)
            flag = self.robot.GetEnv().CheckCollision(self.robot)
            flag = not flag
        return flag 
    #help function
    def ComputePathLength(self, path):
        length = 0

        for milestone in range(1,len(path)):
            dist =  numpy.linalg.norm(numpy.array(path[milestone-1])-numpy.array(path[milestone]))
            length += dist
        return length
    def Extend(self, start_config, end_config):
        with self.robot:        
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #The function should return
        #either None or a configuration such that the linear interpolation between the start configuration and
        #this configuration is collision free and remains inside the environment boundaries.
        #self.boundary_limits = [[-5., -5.], [5., 5.]]
            collision = False
            outside = False
            lower_limits, upper_limits = self.boundary_limits
            resolution = 1000;
            move_dir = [float(end_config[0]-start_config[0])/float(resolution), float(end_config[1]-start_config[1])/float(resolution)];
            position_unchange = self.robot.GetTransform();
            for i in range(resolution+1):  
                #get the robot position
                position = self.robot.GetTransform()
                position[0][3] = start_config[0] + i*move_dir[0];
                position[1][3] = start_config[1] + i*move_dir[1];
                self.robot.SetTransform(position);
                if(self.robot.GetEnv().CheckCollision(self.robot)): 
                    collision = True
                    #print position
                #print "Detected collision!! Will return last safe step"
                #print 
                if( upper_limits[0] < position[0][3] or position[0][3] < lower_limits[0] or upper_limits[1] < position[1][3] or position[1][3] < lower_limits[1]): 
                    outside = True
                #print position
                #print "Detected outside of bound!! Will return last safe step"
                self.robot.SetTransform(position_unchange);
        #check the collision == not touch the table && inside the boundary
                if(collision or outside):
                    if(i >=1):
                        valid_config = [0,0]
                        valid_config[0] = start_config[0] + (i-1)* move_dir[0]
                        valid_config[1] = start_config[1] + (i-1)* move_dir[1]
                        return valid_config
                    else: return None
        #No interpolate with collision and outside
        
        return end_config
                #return NULL
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        
        #
        # TODO: Generate and return a random configuration
        #
        #Peter: just random config in the limitations
        config[0] = lower_limits[0] + (upper_limits[0]-lower_limits[0]) * numpy.random.random_sample()
        config[1] = lower_limits[1] + (upper_limits[1]-lower_limits[1]) * numpy.random.random_sample()
        

        return numpy.array(config)