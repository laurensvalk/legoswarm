import numpy as np

class Spring():
    def __init__(self, characteristic):
        """Extract data from spring characteristic from settings"""
        self.characteristic = np.array(characteristic)
        self.extensions = self.characteristic[:,0]
        self.forces     = self.characteristic[:,1]

    def get_force_magnitude(self, length):
        """Convert a given spring length to a force"""
        return np.interp(length, self.extensions, self.forces)

    def get_force_vector(self, spring_vector):
        """Convert a given spring vector to a force vector"""

        # A negligible length (one micron) to avoid zero divisions
        micron = 0.0001 

        # Obtain length and direction of spring 
        length = np.linalg.norm(spring_vector)+micron
        direction = spring_vector/length
        
        # Force magnitude
        force_magitude = self.get_force_magnitude(length)     

        # Return force vector
        return direction*force_magitude