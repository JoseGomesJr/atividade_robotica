import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base.transforms3d import transl, trotz
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

# Parâmetros do manipulador
parametros = [1, 2, 1, 0.3]
comprimento_base = 5
comprimento_elo1 = 1
comprimento_elo2 = 2
deslocamento1 = 1
deslocamento3 = 3
deslocamento4 = 2

# Crie o manipulador
ARTICU1 = RevoluteDH(a=comprimento_elo1, d=deslocamento1)
ARTICU2 = RevoluteDH(a=comprimento_elo2, alpha=np.pi)
ARTICU3 = PrismaticDH(qlim=[0, deslocamento3])
ARTICU4 = RevoluteDH(d=deslocamento4)
Manipulador = DHRobot([ARTICU1, ARTICU2, ARTICU3, ARTICU4], name='RRPR')

# Letra A

# Crie uma grade de pontos no espaço
x_range = np.linspace(-4, 4, 50)
y_range = np.linspace(-4, 4, 50)
end_effector_positions = []

for x in x_range:
    for y in y_range:
        T = SE3(x, y, 0) * SE3.Rx(np.pi)
        q = Manipulador.ikine_LM(T)
        if q.success:
            end_effector_positions.append([x, y])


end_effector_positions = np.array(end_effector_positions)
# print(end_effector_positions)
# Plote o espaço de trabalho
figura = plt.figure()
eixos = figura.add_subplot(111)

eixos.set_xlabel('x')
eixos.set_ylabel('y')

eixos.set_xlim([-7, 7])
eixos.set_ylim([-7, 7])

eixos.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], s=20)
plt.show()

# Letra B


def ikine_rr(robot, x, y, z):
    # Calcule a cinemática inversa para alcançar a posição (x, y)
    T_desired = SE3(x, y, z)
    q_solutions = robot.ikine_LM(T_desired)
    return q_solutions.q

# Letra C

# No manipulador RR, a orientação do efetuador final é determinada 
# exclusivamente pela combinação dos ângulos q. Não é possível 
# especificar diretamente a orientação do efetuador final 
# independentemente da configuração das juntas.

