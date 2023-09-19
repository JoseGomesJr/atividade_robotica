from setup import *
import math

R = rot2(0.3)
trplot2(R)
tranimate2(R)
plt.show()
vecto = [1,2]

vector_rot = np.dot(R, vecto)
print(vector_rot)

R_inverse = np.transpose(R)
trplot2(R_inverse)
tranimate2(R_inverse)
plt.show()

result_vector = np.dot(R_inverse, vector_rot)
result_rot = np.dot(R_inverse, R)
print(result_vector)
print(result_rot)

determin_r= np.linalg.det(R)
determin_r_inver = np.linalg.det(R_inverse)

print(determin_r)
print(determin_r_inver)


R = rotx(np.pi / 2) @ roty(np.pi / 2)
trplot(R)
tranimate(R)
plt.show()
vecto = [1,2, 3]

vector_rot = np.dot(R, vecto)
print(vector_rot)

R_inverse = np.transpose(R)
trplot(R_inverse)
tranimate(R_inverse)
plt.show()

result_vector = np.dot(R_inverse, vector_rot)
result_rot = np.dot(R_inverse, R)
print(result_vector)
print(result_rot)

determin_r= np.linalg.det(R)
determin_r_inver = np.linalg.det(R_inverse)

print(determin_r)
print(determin_r_inver)