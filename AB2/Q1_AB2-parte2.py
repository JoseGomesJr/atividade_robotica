import math as m
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj, DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3

def produto_vetorial(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)

def robo_duas_articulacoes(comprimento1=1, comprimento2=1):
    return DHRobot([RevoluteDH(a=comprimento1), RevoluteDH(a=comprimento2)], name='RoboDuasArticulacoes')

def calcular_jacobiana(theta1, theta2, comprimento1=1, comprimento2=1):
    x = comprimento1 * m.cos(theta1) + comprimento2 * m.cos(theta1 + theta2)
    y = comprimento1 * m.sin(theta1) + comprimento2 * m.sin(theta1 + theta2)

    J11 = -comprimento1 * m.sin(theta1) - comprimento2 * m.sin(theta1 + theta2)
    J12 = -comprimento2 * m.sin(theta1 + theta2)
    J21 = comprimento1 * m.cos(theta1) + comprimento2 * m.cos(theta1 + theta2)
    J22 = comprimento2 * m.cos(theta1 + theta2)

    J = np.array([[J11, J12], [J21, J22]])
    return J

def robo_duas_articulacoes_controle():
    robo = robo_duas_articulacoes()
    angulos_iniciais = np.array([-np.pi / 2, np.pi / 4])
    pose_final1 = robo.fkine(angulos_iniciais)
    pose_final2 = SE3.Trans(-0.2, 0.3, 0) @ pose_final1
    passos_tempo = np.arange(0, 2, 0.1)
    trajetoria = ctraj(pose_final1, pose_final2, passos_tempo)
    angulos = np.zeros((len(passos_tempo), 2))

    theta1 = np.pi / 2
    theta2 = np.pi / 2
    tamanho_passo_tempo = 0.01
    posicao_desejada = np.array([0, 2])
    comprimento1 = 1
    comprimento2 = 1
    erro_x = []
    erro_y = []
    tempos = []

    for i in range(1000):
        posicao_atual = np.array([comprimento1 * m.cos(theta1) + comprimento2 * m.cos(theta1 + theta2),
                                comprimento1 * m.sin(theta1) + comprimento2 * m.sin(theta1 + theta2)])
        erro_x.append(posicao_desejada[0] - posicao_atual[0])
        erro_y.append(posicao_desejada[1] - posicao_atual[1])
        tempos.append(tamanho_passo_tempo * i)
        J = calcular_jacobiana(theta1, theta2)
        velocidades_efetuador = np.linalg.pinv(J).dot([erro_x[-1], erro_y[-1]])
        theta1 += velocidades_efetuador[0] * tamanho_passo_tempo
        theta2 += velocidades_efetuador[1] * tamanho_passo_tempo
        magnitude_erro = m.sqrt(erro_x[-1]**2 + erro_y[-1]**2)
        if magnitude_erro < 0.01:
            print("Alcançou a posição.")
            break

    robo.teach([theta1, theta2])
    plt.plot(tempos, erro_x)
    plt.title("Erro de X ao longo do Tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de X")
    plt.show()
    plt.title("Erro de Y ao longo do Tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de Y")
    plt.plot(tempos, erro_y)
    plt.show()
    posicao_final = robo.fkine(angulos_iniciais).t[:2]
    print(np.linalg.norm(posicao_final - trajetoria[-1].t[:2]))

robo_duas_articulacoes_controle()
