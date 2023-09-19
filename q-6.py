import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.animation import FuncAnimation
from spatialmath import SE3

def draw_cube(ax, vertices, edges):
    ax.add_collection3d(Poly3DCollection([vertices], facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.1))
    ax.add_collection3d(Line3DCollection(edges, colors='k', linewidths=side))

def animate_rotation(fig, ax, vertices, angles_sequence_x, angles_sequence_y, angles_sequence_z):
    def animate(frame):
        ax.cla()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        angle_x = angles_sequence_x[frame]
        angle_y = angles_sequence_y[frame]
        angle_z = angles_sequence_z[frame]

        transformation = SE3.Rx(angle_x) * SE3.Ry(angle_y) * SE3.Rz(angle_z)
        transformed_vertices = (transformation * vertices.T).T

        edges = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
    ]
        draw_cube(ax, transformed_vertices, edges)

    num_frames = min(len(angles_sequence_x), len(angles_sequence_y), len(angles_sequence_z))
    anim = FuncAnimation(fig, animate, frames=num_frames, interval=100)
    plt.show()

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    side = 0.2
    vertices = np.array([[side, side, side],
                         [side, side, -side],
                         [side, -side, side],
                         [side, -side, -side],
                         [-side, side, side],
                         [-side, side, -side],
                         [-side, -side, side],
                         [-side, -side, -side]])


    angles_x = np.linspace(0, 2 * np.pi, 100)
    angles_y = np.linspace(0, 2 * np.pi, 100)
    angles_z = np.linspace(0, 2 * np.pi, 100)

    animate_rotation(fig, ax, vertices, angles_x, angles_y, angles_z)
