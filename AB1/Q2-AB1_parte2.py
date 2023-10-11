import math
import numpy as np
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt
from spatialmath.base import *


L1 = 1.5
L2 = 1
L3 = 4
L4 = 1.5

#Letra A:
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim([-1, 5])
ax.set_ylim([-1, 5])
ax.set_zlim([0, 5])

TRANSL_0 = transl(0, 0, 0)

TRANSL_1 = transl(0, 0, L1)

TRANSL_2 = transl(0, 0, L1 + L2) @ trotx(np.pi / 2)

TRANSL_3 = transl(L3, 0, L1 + L2) @ trotx(np.pi / 2)

T0 = transl(L3 + L4, 0, L1 + L2) @ trotx(np.pi)


trplot(TRANSL_0, frame="TRANSL_0", color="gray")
trplot(TRANSL_1, frame="TRANSL_1", color="yellow")
trplot(TRANSL_2, frame="TRANSL_2", color="red")
trplot(TRANSL_3, frame="TRANSL_3", color="green")

trplot(T0, frame="T", color="blue")

plt.show()

# Letra B

DH_1 = RevoluteDH(d = L1 + L2, alpha = np.pi / 2, name = '1')
DH_2 = RevoluteDH(a = L3)
DH_3 = RevoluteDH(a = L4)

dh_robot = DHRobot([DH_1, DH_2, DH_3], name = 'DH')
print(dh_robot)

#Letra C:

DH_FK = dh_robot.fkine_all(q=[0, 0, 0])

print(DH_FK)

dh_robot.teach(q = [0, 0, 0])
dh_robot.teach(q = [0, -np.pi/4, np.pi/2])