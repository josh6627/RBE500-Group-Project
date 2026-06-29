import numpy as np


def get_rotation_matrix(axis: int, angle: float) -> np.ndarray:
    if axis == 0:       # X axis
        return np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    elif axis == 1:     # Y axis
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    elif axis == 2:     # Z axis
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0],
                         [0, 0, 1]])

def get_translation_matrix(axis: int, dist: float) -> np.ndarray:
    mat = np.eye(4)             # get identity matrix
    mat[axis, -1] = dist        # adds translation to given axis
    return mat