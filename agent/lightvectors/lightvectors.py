from math import atan2

class vector():

    def __init__(self, list_or_vector):
        self.x, self.y = list_or_vector

    def __str__(self):
        return str(self.aslist)

    @property 
    def aslist(self):
        return [self.x, self.y]

    def __getitem__(self, key):
        return self.aslist[key]

    def __iter__(self):
        for scalar in self.aslist:
            yield scalar

    def __add__(self, right):
        return vector([self.x + right.x, self.y + right.y])

    def __sub__(self, right):
        return vector([self.x - right.x, self.y - right.y])    

    def __neg__(self):
        return vector([-self.x, -self.y])      

    def __truediv__(self, scalar):
        return vector([self.x/scalar, self.y/scalar])

    def __mul__(self, scalar):
        return vector([self.x*scalar, self.y*scalar])

    def __rmul__(self, scalar):
        return vector([self.x*scalar, self.y*scalar])

    @property
    def norm(self):
        return (self.x*self.x + self.y*self.y)**0.5

    @property
    def unit(self):
        return vector(self/self.norm)

    @property
    def angle_with_y_axis(self):
        return -atan2(self.x, self.y) #CCW is positive
