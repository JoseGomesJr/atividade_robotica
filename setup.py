"""
Representando rotações e translações - Python Robotics Toolbox.
"""

import numpy as np
import matplotlib.pyplot as plt

from spatialmath.base import *


def rot_2d(angle=0.3, p=np.array([0.4, 0.4])):
    """
    Rotaciona um ponto em 2D.
    """
    R = rot2(angle) # criando uma matriz de rotação SO(2)
    trplot2(R) # visualizando como um frame rotacionado
    tranimate2(R) # animando a rotação
    plot_point(p, "ko", text="P")
    plotvol2([-1, 1, -1, 1]) # ajustando limites do plot
    plt.show()


def htransform_2d():
    """
    Matriz de transformação homogênea 2D
    """
    T0 = transl2(0, 0)
    TA = transl2(1, 2) @ trot2(30, "deg")
    TB = transl2(2, 1)
    TAB = TA@TB
    P = np.array([3, 2])

    plotvol2([0, 5]) # new plot with both axes from 0 to 5
    trplot2(T0, frame="O", color="k")
    trplot2(TA, frame="A", color="b")
    trplot2(TB, frame="B", color="r")
    trplot2(TAB, frame="AB", color="g")
    plot_point(P, "ko", text="P")

    plt.show()


def rot_frame():
    """
    Rotacionando um sistema de coordenadas
    """
    # %%
    plotvol2([-5, 4, -1, 5])
    T0 = transl2(0, 0)
    trplot2(T0, frame="0", color="k")
    TX = transl2(2, 3)
    trplot2(TX, frame="X", color="b")
    TR = trot2(2)
    trplot2(TR @ TX, frame="RX", color="g")
    trplot2(TX @ TR, frame="XR", color="g")

    # Rotacionando em torno de um ponto C

    C = np.array([3, 2])
    plot_point(C, "ko", text="C")
    TC = transl2(C) @ TR @ transl2(-C)
    trplot2(TC @ TX, frame="XC", color="r")

    plt.show()


def rot_3d():
    """
    Matriz de rotação 3D.
    """
    R = rotx(np.pi / 2) @ roty(np.pi / 2)
    # trplot(R)
    tranimate(R)
    plt.show()


def htransform_3d():
    """
    Matriz de transformação homogênea 3D.
    """
    T0 = transl(0, 0, 0)
    T = transl(3, 0, 0) @ trotx(np.pi / 2) @ transl(0, 4, 0)
    P = transl(1, 3, 1)

    trplot(T0, frame="O", color="k")
    trplot(T, frame="A", color="b")
    trplot(P, frame="P", color="r", length=0, axislabel=False)

    plt.show()
