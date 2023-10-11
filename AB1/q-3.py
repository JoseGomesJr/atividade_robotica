from setup import *
from spatialmath import SO2, SO3, SE2, SE3, UnitQuaternion, Twist3
from spatialmath.base import *

R_A = rotz(np.pi / 9)
R_B = rotx(np.pi / 4) @ roty(np.pi / 8)
trplot(R_A, color="r", frame="A")
trplot(R_B, color="g", frame="B")
plt.show()


R_ARB =  np.transpose(R_A) @ R_B
R_BRA = np.transpose(R_B) @ R_A

print("Matriz aRb:")
print(R_ARB)

print("Matriz aRb:")
print(R_BRA)
trplot(R_ARB, color="y", frame="aRb")
trplot(R_BRA, color="b", frame="bRa")
plt.show()

ang1, eix1 = tr2angvec(R_ARB)
ang2, eix2 = tr2angvec(R_BRA)

print("\nEixo-ângulo:")
print(ang1, eix1)
print(ang2, eix2)