import numpy as np
import robot.transformations as tr
import robot.control.robot_processing as robot_process
import time

robot_pose = [259.539, 708.927, 521.823, 179.599, -47.714, 109.318]
displacement = [20.756071421929505, 33.18850208422151, 87.11548405325067, 9.439261632682332, 1.6278108723386413, -9.243440744654016]

target_actual = [291.19083204226223, 774.0339007937497, 457.89587882730837, 160.19226836191544, -47.44648946901304, 123.159290221115]
target_actual = np.array(target_actual)

xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

# Optimal angles found
optimal_rz, optimal_rx, optimal_ry = 2, 12, 356

# Define tighter ranges around the optimal angles
rz_range = np.arange(optimal_rz - 5, optimal_rz + 5, 1)
rx_range = np.arange(optimal_rx - 5, optimal_rx + 5, 1)
ry_range = np.arange(optimal_ry - 5, optimal_ry + 5, 1)

min_distance = float('inf')
min_combination = None

for rz_offset in rz_range:
    print(rz_offset)
    for rx_offset in rx_range:
        for ry_offset in ry_range:
            Rx = tr.rotation_matrix(np.radians(11), xaxis)
            Ry = tr.rotation_matrix(np.radians(357), yaxis)
            Rz = tr.rotation_matrix(np.radians(4), zaxis)
            rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)
            fix_axis = -displacement[0], displacement[1], displacement[2], -displacement[3], displacement[4], displacement[5]
            m_offset = robot_process.coordinates_to_transformation_matrix(position=fix_axis[:3], orientation=fix_axis[3:], axes='sxyz')
            displacement_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix
            displacement_position, displacement_orientation = robot_process.transformation_matrix_to_coordinates(displacement_matrix, axes='sxyz')
            m_robot = robot_process.coordinates_to_transformation_matrix(
                position=robot_pose[:3],
                orientation=robot_pose[3:],
                axes='sxyz',
            )
            m_offset = robot_process.coordinates_to_transformation_matrix(
                position=displacement_position,
                orientation=displacement_orientation,
                axes='sxyz',
            )
            m_final = m_robot @ m_offset
            translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')
            target_pose = list(translation) + list(angles_as_deg)
            distance = np.linalg.norm(target_actual - target_pose)
            print(distance)
            if distance < min_distance:
                min_distance = distance
                min_combination = (rz_offset, rx_offset, ry_offset)
                print('rz_offset: ')
                print(rz_offset)
                print('rx_offset: ')
                print(rx_offset)
                print('ry_offset: ')
                print(ry_offset)
                print(distance)

print(f"Minimum distance and corresponding angles: {min_combination}, {min_distance}")


# Minimum distance and corresponding angles: (6.0, 10.5, 358.0), 1.1746655409659812

# Minimum distance and corresponding angles: (5.200000000000006, 10.800000000000004, 357.5999999999996), 1.1519984164904342




>>> connection.get_coordinates()
(True, [288.0, 778.12, 460.534, 160.276, -47.457, 123.507])
>>> connection._send_and_receive("MoveRelL,0,1,1,10,1")
(True, None)
>>> connection._send_and_receive("MoveRelL,0,1,1,10,1")
The command  returned the error code: 20005
(False, None)
>>> connection._send_and_receive("MoveRelL,0,1,1,10,1")
The command MoveRelL returned the error code: 20018
(False, None)
>>> connection._send_and_receive("MoveRelL,0,1,1,10,1")
The command MoveRelL returned the error code: 20018
(False, None)
>>> connection.get_motion_state()
The command MoveRelL returned the error code: 20018
Could not read robot motion state
<MotionState.ERROR: 3>
>>> connection.get_motion_state()
<MotionState.IN_MOTION: 1>
>>> connection.get_motion_state()
<MotionState.FREE_TO_MOVE: 0>
>>> connection.get_motion_state()
<MotionState.FREE_TO_MOVE: 0>
>>> connection._send_and_receive("MoveRelL,0,1,1,10,1")
(True, ['0', '1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1'])
>>> connection.get_motion_state()
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "C:\Users\mTMS\Robot_TMS\robot\robots\elfin\elfin_connection.py", line 325, in get_motion_state
    moving_state = bool(int(params[0]))
ValueError: invalid literal for int() with base 10: ';'














displacement = [-59.350894048401784, 20.52410945847183, -1.2442541010742616, 0.08289484448596757, -0.07836793367505461, 1.0326014621993391]

xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

rz_offsets = np.arange(0, 360, 2)
for rz_offset in rz_offsets:
    rx_offset = 0
    ry_offset = 0
    Rx = tr.rotation_matrix(np.radians(rx_offset), xaxis)
    Ry = tr.rotation_matrix(np.radians(ry_offset), yaxis)
    Rz = tr.rotation_matrix(np.radians(rz_offset), zaxis)
    rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)
    fix_axis = -displacement[0], displacement[1], displacement[2], -displacement[3], displacement[4], displacement[5]
    m_offset = robot_process.coordinates_to_transformation_matrix(position=fix_axis[:3], orientation=fix_axis[3:], axes='sxyz')
    displacement_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix
    displacement_position_transformed, displacement_orientation_transformed = robot_process.transformation_matrix_to_coordinates(displacement_matrix, axes='sxyz')
    print(rz_offset, displacement_position_transformed, displacement_orientation_transformed)


m_robot = robot_process.coordinates_to_transformation_matrix(
    position=robot_pose[:3],
    orientation=robot_pose[3:],
    axes='sxyz',
)
m_offset = robot_process.coordinates_to_transformation_matrix(
    position=displacement_position,
    orientation=displacement_orientation,
    axes='sxyz',
)
m_final = m_robot @ m_offset
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')
target_pose = list(translation) + list(angles_as_deg)
distance = np.linalg.norm(target_actual - target_pose)
print(rz_offset, distance)


coordinates:

[259.72, 732.569, 443.485, 168.505, -53.145, 93.592]

close to target:

displacement stored: [0.005357563897945263, 0.16319084433885678, 0.16332876587164513, -0.0732205453551775, 0.11851623420250175, -0.2966453445216762]
displacement orig: [-0.20900008371057766, 0.15658128255191173, 0.06109287660785512, 0.05081840374696196, 0.006775549468735244, -0.15772323012959194]









import numpy as np
import robot.transformations as tr
import robot.control.robot_processing as robot_process
import time

robot_pose = [189.509, 685.124, 517.242, 177.643,  46.564, -91.675]
displacement = [-0.7797766066858998, 0, 0.49356090230185146, -10.027719443885271, -0.35504907399382757, 0.534612266713389]

target_actual = [189.51,  685.124, 517.242, 167.643,  46.564, -91.674]
target_actual = np.array(target_actual)

m_robot = robot_process.coordinates_to_transformation_matrix(
    position=robot_pose[:3],
    orientation=robot_pose[3:],
    axes='sxyz',
)
m_offset = robot_process.coordinates_to_transformation_matrix(
    position=displacement[:3],
    orientation=displacement[3:],
    axes='sxyz',
)
m_final = m_robot @ m_offset
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

target_estimated = list(translation) + list(angles_as_deg)



# 0 deg: [211.143, 681.241, 515.136, -149.904, -40.242, 59.863])
# 180 deg: [211.143, 681.241, 515.135, 149.904, 40.242, -120.137]
# none: [221.09, 633.458, 558.062, -149.904, -40.242, 59.863]

end_effector_pose = [221.09, 633.458, 558.062, -149.904, -40.242, 59.863]
tcp_offset = [0, 0, 65, 0, 0, 180]

m_end_effector = robot_process.coordinates_to_transformation_matrix(
    position=end_effector_pose[:3],
    orientation=end_effector_pose[3:],
    axes='sxyz',
)

m_tcp_offset = robot_process.coordinates_to_transformation_matrix(
    position=tcp_offset[:3],
    orientation=tcp_offset[3:],
    axes='sxyz',
)

displacement = [-10, 0, 0, 0, 0, 0]
m_displacement = robot_process.coordinates_to_transformation_matrix(
    position=displacement[:3],
    orientation=displacement[3:],
    axes='sxyz',
)

m_final = m_end_effector @ m_tcp_offset @ m_displacement
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

final_pose = list(translation), list(angles_as_deg)

# 0 deg: ([214.97424764034682, 687.8433220514513, 521.5951065425734], [-149.904, -40.242, 59.863])
([214.97424764034682, 687.8433220514513, 521.5951065425734], [149.904, 40.242, -120.137])
# 180 deg: ([207.30942963476778, 674.640475032522, 508.6747584228013], [149.904, 40.242, -120.137])


import robot.transformations as tr
def coordinates_to_transformation_matrix_first_translation(position, orientation, axes='sxyz'):
    """
    Transform vectors consisting of position and orientation (in Euler angles) in 3d-space into a 4x4
    transformation matrix that combines the rotation and translation.
    :param position: A vector of three coordinates.
    :param orientation: A vector of three Euler angles in degrees.
    :param axes: The order in which the rotations are done for the axes. See transformations.py for details. Defaults to 'sxyz'.
    :return: The transformation matrix (4x4).
    """
    a, b, g = np.radians(orientation)
    r_ref = tr.euler_matrix(a, b, g, axes=axes)
    t_ref = tr.translation_matrix(position)
    m_img = tr.multiply_matrices(r_ref, t_ref)
    return m_img


import numpy as np
import robot.transformations as tr
import robot.control.robot_processing as robot_process
import time

#robot_pose = [261.143, 731.242, 565.136, -108.678, -8.434, 20.199]
#displacement = [-90.20530752812644, 14.72221853338741, 7.053189147268768, -23.17523798780097, 0.316236862950353, -27.41968712470541]

robot_pose = [231.296, 698.269, 529.42, -158.153, -44.642, 72.053]
displacement = [-28.134096067344444, 1.3983662831391024, 0.07371983349216293, 0.2143647428528889, -0.153121538165356, 30.47343824871779]

target_actual = [211.143, 681.242, 515.136, -138.678, -28.434, 40.199]
target_actual = np.array(target_actual)

m_robot = robot_process.coordinates_to_transformation_matrix(
    position=robot_pose[:3],
    orientation=robot_pose[3:],
    axes='sxyz',
)

m_offset = robot_process.coordinates_to_transformation_matrix(
    position=displacement[:3],
    orientation=displacement[3:],
    axes='sxyz',
)

m_final = m_robot @ np.linalg.inv(m_offset)
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

target_estimated_old = list(translation) + list(angles_as_deg)

m_offset_new = coordinates_to_transformation_matrix_first_translation(
    position=displacement[:3],
    orientation=displacement[3:],
    axes='sxyz',
)

a, b, g = np.radians(displacement[3:])
r_ref = tr.euler_matrix(a, b, g, axes='sxyz')
t_ref = tr.translation_matrix(displacement[:3])

m_final = m_robot @ r_ref @ t_ref
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

target_estimated_new = list(translation) + list(angles_as_deg)


#initial displacement: ~(0, 0, 0, 0, 0, 0)

connection.move_linear_relative(Axis.X, Direction.POSITIVE, 30) # (-29.74589639369833, -0.8944370533599559, 0.29808917228425, -0.14860388886804443, 0.013656024500337042, 0.17883208059288114)
connection.move_linear_relative(Axis.RZ, Direction.NEGATIVE, 30)

# (-28.134096067344444, 1.3983662831391024, 0.07371983349216293, 0.2143647428528889, -0.153121538165356, 30.47343824871779)

connection.move_linear_relative(Axis.RZ, Direction.NEGATIVE, 30)
connection.move_linear_relative(Axis.X, Direction.POSITIVE, 30)

#displacement: (-29.79181717313213, -6.112553555970857, 0.4616372256973591, 0.05209635433359402, 0.03284788487984773, -9.518823947852017)






import robot.transformations as tr
def coordinates_to_transformation_matrix_first_translation(position, orientation, axes='sxyz'):
    """
    Transform vectors consisting of position and orientation (in Euler angles) in 3d-space into a 4x4
    transformation matrix that combines the rotation and translation.
    :param position: A vector of three coordinates.
    :param orientation: A vector of three Euler angles in degrees.
    :param axes: The order in which the rotations are done for the axes. See transformations.py for details. Defaults to 'sxyz'.
    :return: The transformation matrix (4x4).
    """
    a, b, g = np.radians(orientation)
    r_ref = tr.euler_matrix(a, b, g, axes=axes)
    t_ref = tr.translation_matrix(position)
    m_img = tr.multiply_matrices(r_ref, t_ref)
    return m_img


import numpy as np
import robot.transformations as tr
import robot.control.robot_processing as robot_process
import time

#robot_pose = [261.143, 731.242, 565.136, -108.678, -8.434, 20.199]
#displacement = [-90.20530752812644, 14.72221853338741, 7.053189147268768, -23.17523798780097, 0.316236862950353, -27.41968712470541]

# x-y-z rotations:
#robot_pose = [231.134, 698.035, 527.75, -133.001, -45.875, 52.065]
#displacement = [0, 0, 0, -10, -10, -10]

robot_pose = [231.137, 698.036, 527.754, -123.258, -30.026, 36.435]
displacement = [-4.392332356550014, 0.9600610217339351, 0.99075366401118, -13.579585160839905, -3.575303121815431, -31.198704340888572]

# # x-y rotations:
#robot_pose = [231.137, 698.035, 527.75, -141.566, -52.687, 63.33]
#displacement = [0, 0, 0, -10, -10, 0]

# # only x-rotation:
#robot_pose = [231.137, 698.036, 527.753, -148.115, -44.491, 72.033]
#displacement = [0, 0, 0, -10, 0, 0]

target_actual = [231.134, 698.037, 527.751, -158.115, -44.491, 72.033]
target_actual = np.array(target_actual)

m_robot = robot_process.coordinates_to_transformation_matrix(
    position=robot_pose[:3],
    orientation=robot_pose[3:],
    axes='sxyz',
)

a, b, g = np.radians(displacement[3:])
r_ref = tr.euler_matrix(a, b, g, axes='sxyz')
t_ref = tr.translation_matrix(displacement[:3])

m_final = m_robot @ r_ref @ t_ref
translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

target_estimated_new = list(translation) + list(angles_as_deg)
target_estimated_new























xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

# Optimal angles found
optimal_rz, optimal_rx, optimal_ry = 2, 12, 356

# Define tighter ranges around the optimal angles
rz_range = np.arange(optimal_rz - 5, optimal_rz + 5, 1)
rx_range = np.arange(optimal_rx - 5, optimal_rx + 5, 1)
ry_range = np.arange(optimal_ry - 5, optimal_ry + 5, 1)

min_distance = float('inf')
min_combination = None

for rz_offset in rz_range:
    print(rz_offset)
    for rx_offset in rx_range:
        for ry_offset in ry_range:
            Rx = tr.rotation_matrix(np.radians(11), xaxis)
            Ry = tr.rotation_matrix(np.radians(357), yaxis)
            Rz = tr.rotation_matrix(np.radians(4), zaxis)
            rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)
            fix_axis = -displacement[0], displacement[1], displacement[2], -displacement[3], displacement[4], displacement[5]
            m_offset = robot_process.coordinates_to_transformation_matrix(position=fix_axis[:3], orientation=fix_axis[3:], axes='sxyz')
            displacement_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix
            displacement_position, displacement_orientation = robot_process.transformation_matrix_to_coordinates(displacement_matrix, axes='sxyz')
            m_robot = robot_process.coordinates_to_transformation_matrix(
                position=robot_pose[:3],
                orientation=robot_pose[3:],
                axes='sxyz',
            )
            m_offset = robot_process.coordinates_to_transformation_matrix(
                position=displacement_position,
                orientation=displacement_orientation,
                axes='sxyz',
            )
            m_final = m_robot @ m_offset
            translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')
            target_pose = list(translation) + list(angles_as_deg)
            distance = np.linalg.norm(target_actual - target_pose)
            print(distance)
            if distance < min_distance:
                min_distance = distance
                min_combination = (rz_offset, rx_offset, ry_offset)
                print('rz_offset: ')
                print(rz_offset)
                print('rx_offset: ')
                print(rx_offset)
                print('ry_offset: ')
                print(ry_offset)
                print(distance)

print(f"Minimum distance and corresponding angles: {min_combination}, {min_distance}")

