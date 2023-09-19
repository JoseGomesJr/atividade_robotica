from setup import *
from spatialmath import SO2, SO3, SE2, SE3, UnitQuaternion, Twist3
from spatialmath.base import *

matrix_transform = SE3(transl(4, 3, 1) @ trotx(np.pi / 3) @ transl(2, 5, 0))
print("Transformação:")
print(matrix_transform)

inverse = matrix_transform.inv()
print("Inversa:")
print(inverse)

I = matrix_transform @ inverse
print("Matrix Identidade:")
print(I)