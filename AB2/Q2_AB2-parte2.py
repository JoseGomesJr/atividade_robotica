import math as m
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj, DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3

def produto_vetorial(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)

def robo_tres_articulacoes(L1=1, L2=1, L3=1):
    return DHRobot([RevoluteDH(a=L1), RevoluteDH(a=L2), RevoluteDH(a=L3)], name='RoboTresArticulacoes')

def calcular_matriz_transformacao_01(theta1, L1=1):
    T01 = np.array([[m.cos(theta1), -m.sin(theta1), 0, L1 * m.cos(theta1)],
                    [m.sin(theta1), m.cos(theta1), 0, L1 * m.sin(theta1)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return T01

def calcular_matriz_transformacao_12(theta2, L2=1):
    T12 = np.array([[m.cos(theta2), -m.sin(theta2), 0, L2 * m.cos(theta2)],
                    [m.sin(theta2), m.cos(theta2), 0, L2 * m.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return T12

def calcular_matriz_transformacao_23(theta3, L3=1):
    T23 = np.array([[m.cos(theta3), -m.sin(theta3), 0, L3 * m.cos(theta3)],
                    [m.sin(theta3), m.cos(theta3), 0, L3 * m.sin(theta3)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return T23

def jacobiana_geometrica(q):
    L1 = 1
    L2 = 1
    L3 = 1
    x3 = L1 * np.cos(q[0])
    y3 = L1 * np.sin(q[0])
    x2 = x3 + L2 * np.cos(q[0] + q[1])
    y2 = y3 + L2 * np.sin(q[0] + q[1])
    x1 = x2 + L3 * np.cos(q[0] + q[1] + q[2])
    y1 = y2 + L3 * np.sin(q[0] + q[1] + q[2])
    JP1 = np.array([-y1, x1])
    JP2 = np.array([-y2, x2])
    JP3 = np.array([-y3, x3])
    return np.array([JP1, JP2, JP3]).T

def robo_tres_articulacoes_controle():
    robo = robo_tres_articulacoes()
    q0 = np.array([-np.pi / 4, np.pi / 3, -np.pi / 6])
    TE1 = robo.fkine(q0)
    TE2 = SE3.Trans(0.6, 0.5, 0) @ TE1
    t = np.arange(0, 2, 0.02)
    Ts = ctraj(TE1, TE2, t)
    q = np.zeros((len(t), 3))

    q1 = np.pi / 6
    q2 = np.pi / 4
    q3 = 0
    dt = 0.01
    posicao_desejada = np.array([0.8, 1.6])
    L1 = 0.8
    L2 = 0.8
    L3 = 0.6
    erro_x = []
    erro_y = []
    tempos = []
    
    for i in range(1000):
        posicao_atual = np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3),
                                 L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)])
        erro_x.append(posicao_desejada[0] - posicao_atual[0])
        erro_y.append(posicao_desejada[1] - posicao_atual[1])
        tempos.append(dt * i)
        J = jacobiana_geometrica([q1, q2, q3])
        velocidades = np.linalg.pinv(J).dot([erro_x[-1], erro_y[-1]])
        q1 += velocidades[0] * dt
        q2 += velocidades[1] * dt
        q3 += velocidades[2] * dt
        erro = m.sqrt(erro_x[-1] ** 2 + erro_y[-1] ** 2)
        if erro < 0.01:
            print("Alcançou a posição desejada.")
            break
    robo.teach([q1, q2, q3])
    plt.plot(tempos, erro_x)
    plt.title("X Error over Time")
    plt.xlabel("Time")
    plt.ylabel("X Error")
    plt.show()
    plt.title("Y Error over Time")
    plt.xlabel("Time")
    plt.ylabel("Y Error")
    plt.plot(tempos, erro_y)
    plt.show()

robo_tres_articulacoes_controle()
