from setup import *


#Rotação A
rot_a_x = rotx(np.pi / 2)
#trplot(rot_a_x)
tranimate(rot_a_x)
plt.show()

rot_a_y = rot_a_x @ roty(np.pi / 2)
#trplot(rot_a_y)
tranimate(rot_a_y)

plt.show()

#Rotação B
rot_b_x = roty(np.pi / 2)
#trplot(rot_a_x)
tranimate(rot_b_x)
plt.show()

rot_b_y = rot_b_x @ rotx(np.pi / 2)
#trplot(rot_a_y)
tranimate(rot_b_y)

plt.show()

