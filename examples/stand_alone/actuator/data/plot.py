import matplotlib.pyplot as pyplot
import numpy

#data = numpy.genfromtext('data.csv', delimiter='\t', skip_header=1,
#                     names=['actuated_joint_pos', 'actuated_joint_vel',
#                     'actuated_joint_torque', 'unactuated_joint_pos',
#                     'unactuated_joint_vel', 'unactuated_joint_torque'])

data = numpy.loadtxt('data.csv', delimiter='\t', skiprows=1)

# expected headers:
# 0: actuated_joint_pos 1: actuated_joint_vel 2: actuated_joint_torque
# 3: unactuated_joint_pos 4: unactuated_joint_vel 5: unactuated_joint_torque 
time = range(len(data[:, 0]))

# Joint positions over time
pyplot.plot(time, data[:, 3])
pyplot.plot(time, data[:, 0])

pyplot.xlabel('Timestep')
pyplot.ylabel('Joint position')
pyplot.show()

# Joint velocities over time
pyplot.plot(time, data[:, 4])
pyplot.plot(time, data[:, 1])

pyplot.xlabel('Timestep')
pyplot.ylabel('Joint velocity')
pyplot.show()

# Joint torques over time
pyplot.plot(time, data[:, 5])
pyplot.plot(time, data[:, 2])

pyplot.xlabel('Timestep')
pyplot.ylabel('Joint torque')
pyplot.show()

# Torque vs. velocity (velocity on x-axis)

pyplot.plot(data[:, 4], data[:, 5])
pyplot.plot(data[:, 1], data[:, 2])

pyplot.xlabel('Joint velocity')
pyplot.ylabel('Joint torque')
pyplot.show()
