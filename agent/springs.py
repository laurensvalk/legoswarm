class Spring():
    def __init__(self, characteristic):
        """Extract data from spring characteristic from settings"""
        self.forces = [force for (extension, force) in characteristic]
        self.extensions = [extension for (extension, force) in characteristic]
        self.npoints = len(self.extensions)
        self.min_extension = self.extensions[0]
        self.max_extension = self.extensions[-1]

    def get_force_magnitude(self, length):
        """Convert a given spring length to a force"""
        # Custom implementation of interpolation

        if length <= self.min_extension:
            # if the extension is less than the minimum, take force corresponding to minimum length
            force = self.forces[0]
        elif length >= self.max_extension:
            # if the extension is greater than the maximum, take force corresponding to maximum length
            force = self.forces[-1]
        else:
            # Otherwise, our length is somewhere in between
            for index in range(self.npoints):
                # Loop over the extensions in pairs, looking at two at once
                left = self.extensions[index-1]
                right = self.extensions[index]

                if left <= length <= right:
                    # If the spring extension is in between the two selected ones, interpolate to find the force
                    leftforce = self.forces[index-1]
                    rightforce = self.forces[index]
                    force = leftforce + (length-left)/(right-left)*(rightforce-leftforce)
                    # We already have a force, so we can stop looking
                    break
        
        # Return the result
        return force

    def get_force_vector(self, spring_vector):
        """Convert a given spring vector to a force vector"""

        # A negligible length (one micron) to avoid zero divisions
        micron = 0.0001 

        # Obtain length and direction of spring 
        length = (spring_vector[0]**2+spring_vector[1]**2)**0.5+micron
        direction = spring_vector/length
        
        # Force magnitude
        force_magitude = self.get_force_magnitude(length)     

        # Return force vector
        return direction*force_magitude