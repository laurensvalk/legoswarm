from numpy import ndarray, array, append

ROW, COL = 0, 1

class Transformation:
    """Class to store both translation and rotation"""

    # 2D vectors
    dimension = 2

    # Bottom row is always the same
    __bottom_row = array([[0, 0, 1]]) if dimension == 2 else array([[0, 0, 0, 1]])

    def __init__(self, rotation = None, translation = None, matrix = None):
        """Store translation and rotation"""
        if matrix is None:
            # Store rotation and translation if supplied and make matrix
            self.rotation = rotation
            self.translation = array(translation)
            self.matrix = self.make_matrix(rotation, translation)
            self.inverse_matrix = self.make_inverse_matrix(self.rotation, self.translation)
        else:
            # Extract rotation and translation from existing matrix, and store them
            self.rotation, self.translation = self.dissect_matrix(matrix)    
            self.matrix = matrix
            self.inverse_matrix = self.make_inverse_matrix(self.rotation, self.translation)

    def inverse(self):
        """Return the inverse transformation"""
        return Transformation(matrix=self.inverse_matrix)          

    def __mul__(self, point):
        """Transform point to different frame: new = H@old"""
        # Append 1 to facilitate transformation.
        point_appended = append(array(point), 1)
        transformed_point_appended = self.matrix@point_appended
        # Return the result, without the appended 1
        return transformed_point_appended[0:self.dimension]        

    def __matmul__(self, right): 
        """Create composite transformation from two transformations (self@right)"""   
        return Transformation(matrix=self.matrix@right.matrix)

    def __rmatmul__(self, left):
        """Create composite transformation from two transformations (left@self)"""   
        return Transformation(matrix=left.matrix@self.matrix)

    @staticmethod
    def make_matrix(rotation, translation):
        """Homogeneous transformation matrix"""
        # Cast translation into 2D row vector if not already the case
        translation_fixed = translation.reshape((Transformation.dimension,1))
        # Compose a transformation matrix as follows:
        # |  Rotation   translation  |
        # |  0      0      1         |
        return append(append(rotation, translation_fixed, axis=COL),
                      Transformation.__bottom_row, axis=ROW)        

    @staticmethod
    def make_inverse_matrix(rotation, translation):
        """Inverse transformation matrix"""
        # Cast translation into 2D row vector if not already the case
        translation_fixed = translation.reshape((Transformation.dimension,1))

        # Given a transformation matrix as above, return its inverse as:
        # |  Rotation^T   -Rotation^T@translation  |
        # |  0      0             1                |
        return append(append(rotation.T, -rotation.T@translation_fixed, axis=COL),
                      Transformation.__bottom_row, axis=ROW)

    @staticmethod
    def dissect_matrix(matrix):
        """Extract rotation and translation from a given transformation matrix"""
        n = Transformation.dimension
        rotation = matrix[0:n, 0:n]
        translation = matrix[0:n, n]  
        return rotation, translation  