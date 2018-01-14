from numpy import ndarray, array, append, zeros

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
    def make_matrix(rotation, translation):
        """Homogeneous transformation matrix"""
        # Cast translation into column vector if not already the case
        translation_col = Transformation.column_vector(translation)
        # Compose a transformation matrix as follows:
        # |  Rotation   translation  |
        # |  0      0      1         |
        return append(append(rotation, translation_col, axis=COL),
                      Transformation.__bottom_row, axis=ROW)        

    @staticmethod
    def make_inverse_matrix(rotation, translation):
        """Inverse transformation matrix"""
        # Cast translation into column vector if not already the case
        translation_col = Transformation.column_vector(translation)
        # Given a transformation matrix as above, return its inverse as:
        # |  Rotation^T   -Rotation^T@translation  |
        # |  0      0             1                |
        return append(append(rotation.T, -rotation.T@translation_col, axis=COL),
                      Transformation.__bottom_row, axis=ROW)

    @staticmethod
    def dissect_matrix(matrix):
        """Extract rotation and translation from a given transformation matrix"""
        n = Transformation.dimension
        rotation = matrix[0:n, 0:n]
        translation = matrix[0:n, n]  
        return rotation, translation     


    def __init__(self, rotation = None, translation = None, matrix = None):
        """Store translation and rotation"""
        if matrix is None:
            # Store rotation and translation if supplied and make matrix
            self.rotation = rotation
            self.translation = array(translation)
            self.matrix = Transformation.make_matrix(rotation, translation)
            self.inverse_matrix = Transformation.make_inverse_matrix(rotation, translation)
        else:
            # Extract rotation and translation from existing matrix, and store them
            self.rotation, self.translation = self.dissect_matrix(matrix)    
            self.matrix = matrix
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

