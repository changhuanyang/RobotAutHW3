import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)



    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config

    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            coord[idx] = numpy.ceil((config[idx]-self.lower_limits[idx])/self.resolution)
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for idx in range(self.dimension):
            config[idx] = coord[idx]*self.resolution + self.lower_limits[idx]
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0 
        # c = coord
        for idx in range(self.dimension):
            dim = coord[idx]
            for i in range(idx+1,self.dimension):
                dim = dim*self.num_cells[i]
            node_id = node_id + dim

        return node_id


    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        dim = 1
        for i in range(1,self.dimension):
            dim = dim*self.num_cells[i]
            


        coord = [0] * self.dimension
        index = node_id
        denom_dim = dim
        numer_dim = dim
        for idx in range(self.dimension):
            index = node_id
            numer_dim = dim
           
            for j in range(0,idx):
               index = index - coord[j]*numer_dim
               numer_dim = numer_dim / self.num_cells[j + 1]
           
            # if idx == self.dimension - 1 :
            #     coord[idx] = index % self.num_cells[0]
            # else:
            coord[idx] = numpy.floor(index / denom_dim)
            if idx < self.dimension - 1 :
                denom_dim = denom_dim / self.num_cells[idx+1]
            else:
                denom_dim = 1

        # c = coord
        return coord
        
        
        
