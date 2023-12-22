import numpy as np
import robot.transformations as tr
import robot.control.robot_processing as robot_process
import time

site_config={
    'rx_offset': 10,
    'ry_offset': 20,
    'rz_offset': 30,
}

def OnCoilToRobotAlignment(displacement):
    xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
    rx_offset = site_config['rx_offset']
    ry_offset = site_config['ry_offset']
    rz_offset = site_config['rz_offset']
    Rx = tr.rotation_matrix(np.radians(rx_offset), xaxis)
    Ry = tr.rotation_matrix(np.radians(ry_offset), yaxis)
    Rz = tr.rotation_matrix(np.radians(rz_offset), zaxis)
    rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)
    fix_axis = -displacement[0], displacement[1], displacement[2], -displacement[3], displacement[4], displacement[5]
    m_offset = robot_process.coordinates_to_transformation_matrix(
        position=fix_axis[:3],
        orientation=fix_axis[3:],
        axes='sxyz',
    )
    print(m_offset)
    print(rotation_alignment_matrix)
    displacement_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix
    return robot_process.transformation_matrix_to_coordinates(displacement_matrix, axes='sxyz')

def OnCoilToRobotAlignment2(displacement):
    displacement[0] = -displacement[0]
    displacement[3] = -displacement[3]
    rx_offset = site_config['rx_offset']
    ry_offset = site_config['ry_offset']
    rz_offset = site_config['rz_offset']
    tcp_offset = [0, 0, 0, rx_offset, ry_offset, rz_offset]
    tcp_offset_matrix = robot_process.coordinates_to_transformation_matrix(
        position=tcp_offset[:3],
        orientation=tcp_offset[3:],
        axes='sxyz',
    )
    displacement_matrix = robot_process.coordinates_to_transformation_matrix(
        position=displacement[:3],
        orientation=displacement[3:],
        axes='sxyz',
    )
    print(displacement_matrix)
    print(tcp_offset_matrix)
    final_displacement_matrix = np.linalg.inv(tcp_offset_matrix) @ displacement_matrix @ tcp_offset_matrix
    return robot_process.transformation_matrix_to_coordinates(final_displacement_matrix, axes='sxyz')


print(OnCoilToRobotAlignment([1,2,3,4,5,6]))
print(OnCoilToRobotAlignment2([1,2,3,4,5,6]))
 
#(array([ 3.40890187,  0.37533819, -1.49616486]), array([ 7.39004052, -0.212696  , -4.95569958]))