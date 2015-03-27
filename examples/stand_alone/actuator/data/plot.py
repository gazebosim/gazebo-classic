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
passive = pyplot.plot(time, data[:, 3], label='Passive')
actuated = pyplot.plot(time, data[:, 0], label='Actuated')
pyplot.legend(loc='upper left')
pyplot.xlabel('Time')
pyplot.ylabel('Joint position')
pyplot.savefig("actuator_pos_time.png")
pyplot.show()

# Joint velocities over time
pyplot.plot(time, data[:, 4], label='Passive')
pyplot.plot(time, data[:, 1], label='Actuated')

pyplot.legend(loc='upper left')
pyplot.xlabel('Time')
pyplot.ylabel('Joint velocity')
pyplot.savefig("actuator_vel_time.png")
pyplot.show()

# Joint torques over time
pyplot.plot(time, data[:, 5], label='Passive')

pyplot.legend()
pyplot.xlabel('Time')
pyplot.ylabel('Joint torque')
pyplot.savefig("actuator_torque_time_passive.png")
pyplot.show()

pyplot.plot(time, data[:, 2], label='Actuated', color='g')
pyplot.legend()
pyplot.xlabel('Time')
pyplot.ylabel('Joint torque')
pyplot.savefig("actuator_torque_time_actuated.png")
pyplot.show()

# Torque vs. velocity (velocity on x-axis)

pyplot.plot(data[:, 4], data[:, 5], label='Passive')

pyplot.legend()
pyplot.xlabel('Joint velocity')
pyplot.ylabel('Joint torque')
pyplot.xlim(0, max(data[:, 4]))
pyplot.savefig("actuator_torque_speed_passive.png")
pyplot.show()

pyplot.plot(data[:, 1], data[:, 2], label='Actuated', color='g')
pyplot.legend()
pyplot.xlabel('Joint velocity')
pyplot.ylabel('Joint torque')
pyplot.xlim(0, max(data[:, 1]))

pyplot.savefig("actuator_torque_speed_actuated.png")
pyplot.show()
