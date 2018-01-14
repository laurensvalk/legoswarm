from numpy import ndarray, array, append, zeros
from numpy.linalg import det

ROW, COL = 0, 1

class Transformation:
    """Class to store both translation and rotation"""

    # 2D vectors
    dimension = 2

    # Bottom row is 0 0 1 for 2D or 0 0 0 1 for 3D
    __bottom_row = append(zeros(dimension),1).reshape((1, -1))

    @staticmethod
    def column_vector(vector):
        """Cast list or array as explicit column vector of same length"""
        # 1 Column with automatically detected number of rows
        return array(vector).reshape((-1,1)) 

    @staticmethod
    def row_vector(vector):
        """Cast list or array as explicit row vector of same length"""
        # 1 Row with automatically detected number of rows 
        return array(vector).reshape((1,-1))    

    @staticmethod
    def make_matrix(rotation, translation, scaling=1):
        """Homogeneous transformation matrix"""
        # Cast translation into column vector if not already the case
        translation_col = Transformation.column_vector(translation)
        # Compose a transformation matrix as follows:
        # |  scaling*Rotation   scaling*translation  |
        # |  0                  1                    |
        return append(append(scaling*rotation, scaling*translation_col, axis=COL),
                      Transformation.__bottom_row, axis=ROW)        

    @staticmethod
    def make_inverse_matrix(rotation, translation, scaling=1):
        """Inverse transformation matrix"""
        # Cast translation into column vector if not already the case
        translation_col = Transformation.column_vector(translation)
        # Given a transformation matrix as above, return its inverse as:
        # |  1/scaling*Rotation^T   -Rotation^T@translation  |
        # |  0                       1                |
        return append(append(rotation.T/scaling, -rotation.T@translation_col, axis=COL),
                      Transformation.__bottom_row, axis=ROW)

    @staticmethod
    def dissect_matrix(matrix):
        """Extract rotation and translation from a given transformation matrix"""
        # Vector dimension equals that of the transformation, minus the appended 1
        n = matrix.shape[ROW]-1

        # Obtain the scaling factor relative to a identity matrix whose determinant is 1
        scaling = abs(det(matrix))**(1/n)

        # Obtain the rotation and translation, accounted for scaling
        rotation = matrix[0:n, 0:n]/scaling
        translation = matrix[0:n, n]/scaling

        # Return results
        return rotation, translation, scaling 


    def __init__(self, rotation = None, translation = None, scaling=1, matrix = None):
        """Store translation and rotation"""
        if matrix is None:
            # Store rotation, translation, and scaling if supplied as arguments, and make matrix
            self.rotation = rotation
            self.translation = array(translation)
            self.scaling = scaling
            self.matrix = Transformation.make_matrix(rotation, translation, scaling)
            self.inverse_matrix = Transformation.make_inverse_matrix(rotation, translation, scaling)
        else:
            # Extract rotation, translation, and scaling from matrix supplied as argument, and store them
            self.rotation, self.translation, self.scaling = self.dissect_matrix(matrix)    
            self.matrix = matrix
        
        # For either type of constructor, precompute the inverse
        self.inverse_matrix = Transformation.make_inverse_matrix(self.rotation, self.translation) 

    def inverse(self):
        """Return a new transformation object, whose matrix is the inverse of this one"""
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

