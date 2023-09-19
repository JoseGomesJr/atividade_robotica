import numpy as np
import math
import matplotlib.pyplot as plt
from spatialmath.base.transforms3d import transl, trotz
from spatialmath.base import *
from roboticstoolbox import RevoluteDH, PrismaticDH, DHRobot

def calcular_TRANS(vector, L1, L2, D1, D4):
    Matriz_A = np.matrix([[math.cos(vector[0]), -math.sin(vector[0]),  0, L1 * math.cos(vector[0])],
                        [math.sin(vector[0]),  math.cos(vector[0]),  0, L1 * math.sin(vector[0])],    
                        [0,                    0,                    1, D1],
                        [0,                    0,                    0, 1]])
    
    Matriz_B = np.matrix([[math.cos(vector[1]),  math.sin(vector[1]),  0, L2 * math.cos(vector[1])],
                            [math.sin(vector[1]), -math.cos(vector[1]),  0, L2 * math.sin(vector[1])],  
                            [0,                    0,                   -1.0, 0],
                            [0,                    0,                    0, 1]])
        
    Matriz_C = np.matrix([[1,  0,  0, 0],
                            [0,  1,  0, 0],  
                            [0,  0,  1, vector[2]], 
                            [0,  0,  0, 1]])

    Matriz_D = np.matrix([[math.cos(vector[3]), -math.sin(vector[3]),  0, 0],
                            [math.sin(vector[3]),  math.cos(vector[3]),  0, 0],  
                            [0,                    0,                    1, D4], 
                            [0,                    0,                    0, 1]])
        
    return np.around(np.dot(np.dot(np.dot(Matriz_A, Matriz_B), Matriz_C), Matriz_D), 2)

#Questão 3

parametros = [1, 2, 1, 0.3]
comprimento_base = 5
comprimento_elo1 = 1
comprimento_elo2 = 2
deslocamento1 = 1
deslocamento3 = 3
deslocamento4 = 2

figura = plt.figure()
eixos = figura.add_subplot(111, projection='3d')

eixos.set_xlabel('x')
eixos.set_ylabel('y')
eixos.set_zlabel('z')

eixos.set_xlim([-2, 5])
eixos.set_ylim([-2, 5])
eixos.set_zlim([-2, 5])

TRANS0 = transl(0, 0, 0) @ trotz(np.pi)
TRANS1 = transl(0, 0, comprimento_base) @ trotz(np.pi)
TRANS2 = transl(comprimento_elo1, 0, comprimento_base)
TRANS3 = transl(comprimento_elo1 + comprimento_elo2, 0, comprimento_base + deslocamento1)
TRANS4 = transl(comprimento_elo1 + comprimento_elo2, 0, comprimento_base + deslocamento1 - deslocamento3)
TRANST = transl(comprimento_elo1 + comprimento_elo2, 0, comprimento_base + deslocamento1 - deslocamento3 - deslocamento4) @ trotz(np.pi)

trplot(TRANS0, frame="0", color="k")
trplot(TRANS1, frame="1", color="g")
trplot(TRANS2, frame="2", color="r")
trplot(TRANS3, frame="3", color="k")
trplot(TRANS4, frame="4", color="b")
trplot(TRANST, frame="T", color="k")

plt.show()

ARTICU1 = RevoluteDH(a=comprimento_elo1, d=deslocamento1)
ARTICU2 = RevoluteDH(a=comprimento_elo2, alpha=np.pi)
ARTICU3 = PrismaticDH(qlim=[0, deslocamento3])
ARTICU4 = RevoluteDH(d=deslocamento4)
Manipulador = DHRobot([ARTICU1, ARTICU2, ARTICU3, ARTICU4], name='RRPR')
print(Manipulador)

TRANS_calculada = calcular_TRANS(parametros, comprimento_elo1, comprimento_elo2, deslocamento1, deslocamento4)
print("Transformação Calculada:")
print(TRANS_calculada)

print("Transformação DHRobot Fkine:")
print(Manipulador.fkine(parametros))

Manipulador.teach(parametros)
