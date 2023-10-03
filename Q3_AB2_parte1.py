import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

def ikine_scara(x, y, z, L1, L2, D2, D4):

    radius = np.sqrt(x**2 + y**2)
    if radius > (L1 + L2) or radius < abs(L1 - L2):
        print("Coordenadas fora dos limites de trabalho.")
        return None

    theta1 = np.arctan2(y, x)


    L3 = np.sqrt(x**2 + y**2) - D2

    cos_theta2 = (L3**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)

    z3 = z - D4
    theta3 = z3

    return theta1, theta2, theta3

# Exemplo de uso:
x_desired = 0.8
y_desired = 0.5
z_desired = 0.7
L1 = 1.0
L2 = 1.0
D2 = 0.2
D4 = 0.1

theta1, theta2, theta3 = ikine_scara(x_desired, y_desired, z_desired, L1, L2, D2, D4)

revL1 = RevoluteDH(a = L1, d = 0.2)
revL2 = RevoluteDH(a = L2, alpha = np.pi)
priDH = PrismaticDH(qlim = [0, 1])
revD4 =  RevoluteDH(d = D4)
rob = DHRobot([revL1, revL2, priDH, revD4], name = 'RRPR')
rob.teach([theta1, theta2, theta3, 0])