import numpy as np

def rotX(theta):
    """
    Generate a 3D rotation matrix for rotation around the X-axis.
    
    Parameters:
    - theta: Angle in radians for the rotation.
    
    Returns:
    - R: 3x3 rotation matrix.
    """
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[1, 0, 0],
                  [0, c, -s],
                  [0, s, c]])
    return R

def rotY(theta):
    """
    Generate a 3D rotation matrix for rotation around the Y-axis.
    
    Parameters:
    - theta: Angle in radians for the rotation.
    
    Returns:
    - R: 3x3 rotation matrix.
    """
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])
    return R

def rotZ(theta):
    """
    Generate a 3D rotation matrix for rotation around the Z-axis.
    
    Parameters:
    - theta: Angle in radians for the rotation.
    
    Returns:
    - R: 3x3 rotation matrix.
    """
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    return R

def rotMat(theta_x, theta_y, theta_z):
    """
    Combine rotations around X, Y, and Z axes to create a 3D rotation matrix.
    
    Parameters:
    - theta_x: Angle in radians for the rotation around the x-axis
    - theta_y: Angle in radians for the rotation around the y-axis
    - theta_z: Angle in radians for the rotation around the z-axis
    
    Returns:
    - R: 3x3 rotation matrix representing the combined rotation.
    """
    Rx = rotX(theta_x)
    Ry = rotY(theta_y)
    Rz = rotZ(theta_z)
    
    # Combine rotations by multiplying the matrices in the order of Z, Y, X
    R = np.dot(Rz, np.dot(Ry, Rx))
    
    return R


def homogenous_transformation_matrix(dx: float = 0, dy: float = 0, dz: float = 0, rx: float = 0, ry: float = 0, rz: float = 0):
    """
    Return the homogenius transformation matrix of 
    a translation (dx, dy, dz) and 
    a rotation of rx around the x axis, ry around the y axis and rz around the z axis
    """
    R = rotMat(rx, ry, rz)
    p = np.array([dx, dy, dz])

    # Create a 4x4 homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    
    # Copy the rotation matrix into the upper-left 3x3 portion
    homogeneous_matrix[:3, :3] = R
    
    # Copy the translation vector into the last column of the matrix
    homogeneous_matrix[:3, 3] = p
    
    return homogeneous_matrix
