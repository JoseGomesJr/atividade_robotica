import math as m
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj, DHRobot, RevoluteDH, PrismaticDH

def produto_vetorial(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)

def jacobian_scara(q):
    L1 = 1
    L2 = 1
    return np.array([[-L1*m.sin(q[0]) - L2*m.sin(q[0]+q[1]), L1*m.cos(q[0]) + L2*m.cos(q[0] + q[1]), 0],
                     [-L2*m.sin(q[0] + q[1]), L2*m.cos(q[0] + q[1]), 0],
                     [0, 0, 0]])

def robo_rrpr(L1=1, L2=1, D1=0.1, D3=0.3, D4=0.3):
    revL1 = RevoluteDH(a=L1)
    revL2 = RevoluteDH(a=L2, alpha=np.pi)
    priD3 = PrismaticDH(qlim=[0, 1])
    revD4 = RevoluteDH(d=D4)
    rob = DHRobot([revL1, revL2, priD3, revD4], name='RRPR')

    q1 = 0
    q2 = 0
    q4 = 0
    dt = 0.01
    posicao_desejada = np.array([1, 0, 1 - D3 - D4 + D1]) 
    erro_x = []
    erro_z = []
    tempo = []
    for i in range(1000):
        pos_atual = np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2), 0, 1 - D3 - D4 + D1])
        erro_x.append(posicao_desejada[0] - pos_atual[0])
        erro_z.append(posicao_desejada[1] - pos_atual[1])
        tempo.append(dt * i)
        J = jacobian_scara([q1, q2, q4])
        velocidades = np.linalg.pinv(J).dot([erro_x[-1], 0, erro_z[-1]])
        q1 += velocidades[0] * dt
        q2 += velocidades[1] * dt
        q4 += velocidades[2] * dt
        erro = m.sqrt(erro_x[-1]**2 + erro_z[-1]**2)
        if erro < 0.01:
            print("Alcançou a posição desejada.")
            break
    rob.teach([q1, q2, 0, q4])
    plt.plot(tempo, erro_x)
    plt.title("Erro de X em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de X")
    plt.show()
    plt.title("Erro de Z em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de Z")
    plt.plot(tempo, erro_z)
    plt.show()

robo_rrpr()