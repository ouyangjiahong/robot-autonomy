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
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/self.resolution[idx])
        #print "num_cells=",self.num_cells
    
    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for i in range(self.dimension):
        	coord[i] = int((config[i]-self.lower_limits[i])/self.resolution[i])
        return coord

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for i in range(self.dimension):
        	coord[i] = int((config[i]-self.lower_limits[i])/self.resolution[i])
        return coord


    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for i in range(self.dimension):
            if  (coord[i] >= self.num_cells[i]-1):
                config[i] = self.upper_limits[i] - self.lower_limits[i]
            else:
                config[i] =  self.lower_limits[i] + coord[i] * self.resolution[i] + self.resolution[i] / 2
        return config

    def GridCoordToNodeId(self,coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        node_id = 0
        for i in range(self.dimension):
        	tmp = 1
        	for j in range(i+1, self.dimension):
        		tmp *= self.num_cells[j]
        	node_id += coord[i] * tmp
        # node_id = numpy.ravel_multi_index(coord, self.num_cells, order='F')
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        for i in range(self.dimension):
            tmp = 1
            for j in range(i+1, self.dimension):
                tmp *= self.num_cells[j]
            coord[i] = int(node_id // tmp)
            node_id -= coord[i] * tmp
        # coord = numpy.unravel_index(node_id, self.num_cells, order='F')
        return coord
