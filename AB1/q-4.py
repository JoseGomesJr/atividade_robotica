from setup import *
from spatialmath import SO2, SO3, SE2, SE3, UnitQuaternion, Twist3
from spatialmath.base import *

matrix_transform = (transl(0.2, 0, 0) @ trotx(np.pi / 2) @ transl(0.2, 0, 0))
tranimate(matrix_transform)
plt.show()

inverse = np.linalg.inv(matrix_transform)
result = matrix_transform @ inverse
print("Multiplicando matrix pela a sua inversa:")
print(result)

result = inverse @ matrix_transform
print("\nMultiplicando a inversa pela a original:")
print(result)