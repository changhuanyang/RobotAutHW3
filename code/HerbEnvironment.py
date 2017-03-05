import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        #
        #Peter: we have deal with the close to upper limit in DiscreteEnviroment.py/GridCoordToConfiguration
        #so this line will cause confusion
        #upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        #upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        #for idx in range(len(upper_config)):
        #    self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
    
        node_config = numpy.array(self.discrete_env.NodeIdToConfiguration(node_id))
        node_coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))

        #for each direction there are two type neighbor
        for idx in range(self.discrete_env.dimension):
            #positive direction
            if( (node_coord[idx] + 1) <= (self.discrete_env.num_cells[idx]-1) ):
                n_node_coord = numpy.copy(node_coord)
                n_node_coord[idx] += 1
                n_node_id = self.discrete_env.GridCoordToNodeId(n_node_coord)
                successors.append(n_node_id)
            #negation direction
            if( (node_config[idx] - 1) >= 0 ):
                n_node_coord = numpy.copy(node_coord)
                n_node_coord[idx] -= 1
                n_node_id2 = self.discrete_env.GridCoordToNodeId(n_node_coord)
                successors.append(n_node_id2)

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
    
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
       
        #get node_id
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        #calculate distance
        dist = numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        cost = self.ComputeDistance(start_id,goal_id)

        return cost
    def is_collision(self, n_config):
        #change the current position valuse wiht n_config
        self.robot.SetActiveDOFValues(numpy.array(n_config))
        #move robot to new positi
        #print "checkcollision = ",self.robot.GetEnv().CheckCollision(self.robot)
        #print "selfcheck= ",self.robot.CheckSelfCollision()
        return self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()

    def ComputePathLength(self, path):
        length = 0
        for milestone in range(1,len(path)):
            dist =  numpy.linalg.norm(numpy.array(path[milestone-1])-numpy.array(path[milestone]))
            length += dist
        
        return length
