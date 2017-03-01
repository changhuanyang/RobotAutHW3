import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
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

        successors = [0]*4

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        print config
        idx = 0
        if config[0] + self.discrete_env.resolution < self.upper_limits[0]:
            successors[idx] = self.discrete_env.ConfigurationToNodeId([config[0] + self.discrete_env.resolution, config[1]]) 
        else:
            successors[idx] = -1
        idx = idx + 1

        if config[1] + self.discrete_env.resolution < self.upper_limits[1]:
            successors[idx] = self.discrete_env.ConfigurationToNodeId([config[0], config[1] + self.discrete_env.resolution])
        else:
            successors[idx] = -1
        idx = idx + 1

        if config[0] - self.discrete_env.resolution > self.lower_limits[0]:
            successors[idx] = self.discrete_env.ConfigurationToNodeId([config[0] - self.discrete_env.resolution, config[1]]) 
        else:
            successors[idx] = -1
        idx = idx + 1

        if config[1] - self.discrete_env.resolution > self.lower_limits[1]:
            successors[idx] = self.discrete_env.ConfigurationToNodeId([config[0], config[1] - self.discrete_env.resolution])
        else:
            successors[idx] = -1
        idx = idx + 1



        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = numpy.linalg.norm(numpy.array(self.discrete_env.NodeIdToConfiguration(start_id)) - numpy.array(self.discrete_env.NodeIdToConfiguration(end_id)))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = self.ComputeDistance(start_id,goal_id)

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

        
