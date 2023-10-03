import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

L1 = 0.15
L2 = 0.15
L3 = 0.15
# Letra A
q1_range = np.linspace(-np.pi/2, np.pi/2, 50)
q2_range = np.linspace(-np.pi/2, np.pi/2, 50)
q3_range = np.linspace(-np.pi/2, np.pi/2, 50)

ARTICU1 = RevoluteDH(a=0, alpha=-np.pi/2, offset=0)
ARTICU2 = RevoluteDH(a=L2, alpha=0, offset=0)
ARTICU3 = RevoluteDH(a=L3, alpha=0, offset=0)
Manipulador = DHRobot([ARTICU1, ARTICU2, ARTICU3], name='Perna_Robotica')

end_effector_positions = []

for q1 in q1_range:
    for q2 in q2_range:
        for q3 in q3_range:
            q = [q1, q2, q3]
            T = Manipulador.fkine(q)
            end_effector_positions.append(T.A[0:3, 3])

end_effector_positions = np.array(end_effector_positions)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2], s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Espaço de Trabalho da Perna Robótica')
plt.show()

# Letra B

def ikine_leg(x, y, z):
    L1 = 0.15
    L2 = 0.15
    L3 = 0.15

    q1 = np.arctan2(y, x)

    r = np.sqrt(x**2 + y**2)

    cos_q2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_q2 = np.sqrt(1 - cos_q2**2)
    q2 = np.arctan2(sin_q2, cos_q2)

    q3 = np.arctan2(z - 0.15, r)

    return [q1, q2, q3]


ARTICU1 = RevoluteDH(a=0, alpha=-np.pi/2, offset=0)
ARTICU2 = RevoluteDH(a=L2, alpha=0, offset=0)
ARTICU3 = RevoluteDH(a=L3, alpha=0, offset=0)
Manipulador_DH = DHRobot([ARTICU1, ARTICU2, ARTICU3], name='Perna_Robotica_DH')

x_desired = 0.1
y_desired = 0.1
z_desired = 0.1

q_ikine = ikine_leg(x_desired, y_desired, z_desired)

T_desired = Manipulador_DH.fkine(q_ikine)
q_DH = Manipulador_DH.ikine_LM(T_desired)

print("Solução da cinemática inversa:", q_ikine)
print("Solução da cinemática inversa do RobotDH:", q_DH)

# Letra C