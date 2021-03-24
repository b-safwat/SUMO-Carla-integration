import numpy as np


def get_vector_from_2points(pt1, pt2):
    return [pt2[0]-pt1[0], pt2[1]-pt1[0]]


def get_vector_from_vector_and_angle(vector, angle_degree):
    angle_radian = angle_degree*np.pi/180
    mag_v = np.sqrt(vector[0]*vector[0] + vector[1]*vector[1])

    vector = [vector[0]/mag_v, vector[1]/mag_v]
    # (x cos alpha + y sin alpha, -x sin alpha + y cos alpha)
    return [vector[0]* np.cos(angle_radian) + vector[1] * np.sin(angle_radian)*-1,
            vector[0] * np.sin(angle_radian) + vector[1] * np.cos(angle_radian)]


def euclidean_dist(pt1, pt2):
    return np.sqrt((pt1[0]-pt2[0])*(pt1[0]-pt2[0]) + (pt1[1]-pt2[1])*(pt1[1]-pt2[1]))

def angle_between_2vectors(vec1, vec2):
    angle = np.arctan2(np.cross(vec1, vec2), np.dot(vec1, vec2)) * 180 / np.pi

    if angle < 0:
        angle += 360

    return angle


def pixel_coord_np(width, height):
    """
    Pixel in homogenous coordinate
    Returns:
        Pixel coordinate:       [3, width * height]
    """
    x = np.linspace(0, width - 1, width).astype(np.int)
    y = np.linspace(0, height - 1, height).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))


def intrinsic_from_fov(height, width, fov=90):
    """
    Basic Pinhole Camera Model
    intrinsic params from fov and sensor width and height in pixels
    Returns:
        K:      [4, 4]
    """
    px, py = (width / 2, height / 2)
    hfov = fov / 360. * 2. * np.pi
    fx = width / (2. * np.tan(hfov / 2.))

    vfov = 2. * np.arctan(np.tan(hfov / 2) * height / width)
    fy = height / (2. * np.tan(vfov / 2.))

    return np.array([[fx, 0, px, 0.],
                     [0, fy, py, 0.],
                     [0, 0, 1., 0.],
                     [0., 0., 0., 1.]])


def image_to_cam_coordinate(depth_img):
    height, width = depth_img.shape

    K = intrinsic_from_fov(height, width, 90)  # +- 45 degrees
    K_inv = np.linalg.inv(K)

    # Get pixel coordinates
    pixel_coords = pixel_coord_np(width, height)  # [3, npoints]

    # Apply back-projection: K_inv @ pixels * depth
    cam_coords = K_inv[:3, :3] @ pixel_coords * depth_img.flatten()
    cam_coords = cam_coords.T
    cam_coords = cam_coords.reshape((height, width, 3))
    # back-projection using native for-loop.
    # Uncomment block to test this
    # cam_coords = np.zeros((height, width, 3))
    # u0 = K[0, 2]
    # v0 = K[1, 2]
    # fx = K[0, 0]
    # fy = K[1, 1]
    # i = 0
    # # Loop through each pixel in the image
    # for v in range(height):
    #     for u in range(width):
    #         # Apply equation in fig 3
    #         x = (u - u0) * depth_img[v, u] / fx
    #         y = (v - v0) * depth_img[v, u] / fy
    #         z = depth_img[v, u]
    #         cam_coords[v, u] = (x, y, z)
    #         i += 1
    return cam_coords


if __name__ == '__main__':
    zero_vec = get_vector_from_vector_and_angle([1, 0], -42)
    last_vec = get_vector_from_vector_and_angle(zero_vec, 5*42)
    # print(zero_vec)
    # angle = angle_between_2vectors([1, 0], [0, 1])
    angle = angle_between_2vectors(zero_vec, last_vec)+360

    print(angle)