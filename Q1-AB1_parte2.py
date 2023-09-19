import math
import numpy as np
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt
from spatialmath.base import *

    
L1 = 1
L2 = 1
p = [0.5, 0.5]

B = np.arccos(p[0]**2 + p[1]**2 - L1**2 - L2**2 / 2 * L1 * L2)
A = np.arctan2(p[1], p[0]) - np.arctan2(L2 * np.sin(B), L1 + L2 * np.cos(B))

arm = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)
arm.teach([A, B])

# Letra A
Q_positivo = [-0.424, 2.4188]
Q_negativo = [1.9948, -2.4188]


FK_POSITIVO = arm.fkine(Q_positivo)

print(f"fkine q+:\n{FK_POSITIVO}")

arm.teach(Q_positivo)

FK_NEGATIVO = arm.fkine(Q_negativo)

print(f"fkine q-:\n{FK_NEGATIVO}")
arm.teach(Q_negativo)

# Letra B
L1 = 2.5
L2 = 4

arm_b = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2) * ET2.tx(qlim=[0, 1])
arm_b.teach([A, B, 1])

arm_b.teach([0, B, 2])

# Letra C

q = [0, 0.5, 0.5]
FK = arm_b.fkine(q)

print(f"fkine ={FK}")
arm_b.teach(q)