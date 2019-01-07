###############################################################################

import numpy as np

################################################################################

def calculate_cte(final_waypoints, current_pose):
    """
	the CTE is computed as per the MPC project. The final trajectory is computed using the waypoints
	and a polynomial is fit.

    """
    origin = final_waypoints[0].pose.pose.position
    offset = 11

    # Fetch the waypoints ( They are in world coordinate system )
    waypoints_as_matrix = waypoint_coordinates(final_waypoints)

    # Points are shifted to the origin ( Convert them to car coordinate system )
    shifted_matrix = \
		waypoints_as_matrix - np.array([origin.x, origin.y])

    rot_angle = np.arctan2(shifted_matrix[11, 1], shifted_matrix[11, 0])
    
    # Rotate the matrix using the rotation angle, and then fit a polynomial of degree 2
    rot_matrix = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], [np.sin(rot_angle), np.cos(rot_angle)]])
    rot_matrix_after_dot = np.dot(shifted_matrix, rot_matrix)
    coefficients = np.polyfit(rot_matrix_after_dot[:, 0], rot_matrix_after_dot[:, 1], 2)

    
    # Now converted the coor to car view
    pose_after_shifting = np.array([current_pose.pose.position.x - origin.x, \
				current_pose.pose.position.y - origin.y])
    pose_after_rotation = np.dot(pose_after_shifting, rot_matrix)

    desired_y = np.polyval(coefficients, pose_after_rotation[0])
    current_y = pose_after_rotation[1]

    # compute cte
    cte = desired_y - current_y

    return cte

###############################################################################

def waypoint_coordinates(waypoints_world_coor):
    return list(map(lambda waypoints_world_coor: [waypoints_world_coor.pose.pose.position.x, \
		waypoints_world_coor.pose.pose.position.y], waypoints_world_coor))

###############################################################################

