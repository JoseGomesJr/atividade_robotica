import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base.transforms3d import transl, trotz
from spatialmath.base import *
from roboticstoolbox import RevoluteDH, PrismaticDH, DHRobot

x = 0
y = 0.5
z = -0.5
L1 = 1
L2 = 1

############################Letra A############################
points_L1_x = []
points_L1_y = []
points_L1_z = []
points_L2_x = []
points_L2_y = []
points_L2_z = []

samples = 8

angq1 = np.linspace(0, np.pi, samples)
angq2 = np.linspace(0, np.pi, samples)
angq3 = np.linspace(0, np.pi, samples)

for i in range(samples):
    for j in range(samples):
        x = 0
        y = 0
        z = 0
        for l in range(11):
            x = - ((L1 * l) / 10)*np.sin(angq2[j])*np.cos(angq1[i])
            y =   ((L1 * l) / 10)*np.sin(angq2[j])*np.sin(angq1[i])
            z = - ((L1 * l) / 10)*np.cos(angq2[j])
            points_L1_x.append(x)
            points_L1_y.append(y)
            points_L1_z.append(z)
        aux_x = x
        aux_y = y
        aux_z = z
        for k in range(samples):
            for l in range(11):
                x = aux_x - ((L2 * l) / 10)*np.sin(angq2[j])*np.cos(angq3[k])
                y = aux_y + ((L2 * l) / 10)*np.sin(angq2[j])*np.sin(angq3[k])
                z = aux_z - ((L2 * l) / 10)*np.cos(angq2[j])
                points_L2_x.append(x)
                points_L2_y.append(y)
                points_L2_z.append(z)

second = plt.figure().add_subplot(111, projection = '3d')

j1_x = L1 * np.cos(angq1)
j1_y = L1 * np.sin(angq1)
second.plot(j1_x, j1_y, 0, label='J1')

j2_x = L2 * np.cos(angq2)
j2_y = np.zeros_like(angq2)
j2_z = L2 * np.sin(angq2)
second.plot(j2_x, j2_y, j2_z, label='J2')

j3_x = (L1 + L2) * np.cos(angq3)
j3_y = (L1 + L2) * np.sin(angq3)
second.plot(j3_x, j3_y, 0, label='J3')

second.scatter(points_L1_x, points_L1_y, points_L1_z, marker = '.', s = 5, c='r')
second.scatter(points_L2_x, points_L2_y, points_L2_z, marker = '.', s = 5, c='b')

plt.title("A: Área de trabalho dos movimentos")
plt.show()

    ############################Letra B############################
def funcao(x, y, z, L1, L2):
    radius = np.sqrt(z**2 + y**2)
    cos3 = (y**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos2 = (y**2 + z**2 + L1**2 - L2**2) / (2 * L1 * radius) if radius != 0 else 0

    if abs(cos3) > 1 or abs(cos2) > 1:
        print("Valor fora dos limites. Valor absoluto do cosseno entre os elos acima de 1")
        return

    sen3 = np.sqrt(1 - cos3**2)
    r31 = np.arctan2(sen3, cos3) + np.pi/2
    r32 = np.arctan2(-sen3, cos3) + np.pi/2
    sen2 = np.sqrt(1 - cos2**2)
    r21 = np.arctan2(z, y) - np.arctan2(sen2, cos2)
    r22 = np.arctan2(z, y) + np.arctan2(sen2, cos2)
    r11 = np.arctan2(y, x) - np.pi/2

    if abs(r11) > np.pi/2 or abs(r21) > np.pi/2 or abs(r31) > np.pi/2:
        print("Solução 1:")
        print('θ1 =', r11, 'θ2 =', r21, 'θ3 =', r31)
        e1 = RevoluteDH(d = 0, alpha = np.pi / 2, offset = np.pi / 2)
        e2 = RevoluteDH(a = L1)
        e3 = RevoluteDH(a = L2, offset = -np.pi / 2)
        rob = DHRobot([e1,e2,e3], name = 'RRR')
        rob.teach([r11, r21, r31])
        fkinerob = rob.fkine(q = [r11, r21, r31])
        sol = rob.ikine_LM(fkinerob)
        print('Pose =\n', fkinerob)
        print('Solução 1:\n', sol)
    else:
        print("Valor fora dos limites propostos.")

    if abs(r11 - np.pi/2) > np.pi/2 or abs(r22) > np.pi/2 or abs(r32 - np.pi/2) > np.pi/2:
        print("Solução 2:")
        print('θ1 =', r11, 'θ2 =', r22, 'θ3 =', r32)
        e1 = RevoluteDH(d = 0, alpha = np.pi / 2, offset = np.pi / 2)
        e2 = RevoluteDH(a = L1)
        e3 = RevoluteDH(a = L2, offset = -np.pi / 2)
        rob = DHRobot([e1,e2,e3], name = 'RRR')
        rob.teach([r11, r22, r32]) 
        fkinerob = rob.fkine(q = [r11, r22, r32])
        sol = rob.ikine_LM(fkinerob)
        print('Pose =\n', fkinerob)
        print('Solução 2:\n', sol)
    
    ############################Letra C############################
funcao(0, 0.1, 0.1, 0.15, 0.15)