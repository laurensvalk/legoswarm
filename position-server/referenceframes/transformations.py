# -----------------------------------------------------------------------------
# Copyright (c) 2017 Laurens Valk <laurensvalk@gmail.com>
# -----------------------------------------------------------------------------

from numpy import ndarray, array, append, zeros, ones, eye
from numpy.linalg import det

ROW, COL = 0, 1

class Transformation:
    """Class to store both translation and rotation"""

    # 2D vectors
    dimension = 2

    # Default rotation and translation: None
    no_translation = zeros(dimension)
    no_rotation = eye(dimension)

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


    def __init__(self, rotation=no_rotation, translation=no_translation, scaling=1, matrix=None):
        """Store translation and rotation"""
        if matrix is None:
            # Store rotation, translation, and scaling if supplied as arguments, and make matrix
            self.rotation = rotation
            self.translation = array(translation)
            self.scaling = scaling
            self.matrix = Transformation.make_matrix(rotation, translation, scaling)
        else:
            # Extract rotation, translation, and scaling from matrix supplied as argument, and store them
            self.rotation, self.translation, self.scaling = self.dissect_matrix(matrix)    
            self.matrix = matrix
        
        # For either type of constructor, precompute the inverse
        self.inverse_matrix = Transformation.make_inverse_matrix(self.rotation, self.translation, self.scaling) 

    def inverse(self):
        """Return a new transformation object, whose matrix is the inverse of this one"""
        return Transformation(matrix=self.inverse_matrix)          

    def __mul__(self, points):
        """Transform points to different frame: output_points = H@input_points

        Output has same dimension as input:
            - input_points may be a 1D array representing a vector.
            - input_points may be a 2D column array representing a vector.   
            - input_points may be multiple horizontally concanated vectors        
        """

        # Cast input to array in case a plain list of tuple is used as argument
        points = array(points)

        # Input shape tuple
        input_shape = points.shape

        # Determine if input is a single array or a matrix of multiple arrays
        if len(input_shape) == 1 or 1 in input_shape:
            # The input is a 1D array, or 2D array representing a column vector
            # So append a 1, and convert it to an explicit column vector
            number_of_points = 1
            appended_points = Transformation.column_vector(append(points,1))
        else:
            # The input is a matrix of multiple horizontally concatenated column vectors
            assert input_shape[ROW] == self.dimension, "Dimension error"
            number_of_points = input_shape[COL]
            bottom_row = ones((1, number_of_points))
            appended_points = append(points, bottom_row, axis=ROW)

        # Do the matrix multiplications, giving the transformed
        # coordinates, still with the 1 appended
        transformed_points_appended = self.matrix@appended_points

        # Obtain the result, without the appended 1
        transformed_points = transformed_points_appended[0:self.dimension][:] 

        # Return the result in the same shape as the input argument
        return transformed_points.reshape(input_shape)

    def __matmul__(self, right): 
        """Create composite transformation from two transformations (self@right)"""   
        return Transformation(matrix=self.matrix@right.matrix)

    def __rmatmul__(self, left):
        """Create composite transformation from two transformations (left@self)"""   
        return Transformation(matrix=left.matrix@self.matrix)

