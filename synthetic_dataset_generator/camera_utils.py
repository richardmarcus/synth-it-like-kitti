import logging

import numpy as np
from numpy.linalg import inv
import carla

# TODO Get width and height from the args
WINDOW_WIDTH = 1242  # 1248
WINDOW_HEIGHT = 375  # 384
BB_COLOR = (248, 64, 24)


def get_intrinsic_camera_parameters(camera_blueprint, is_behind_camera=False):
    image_w = camera_blueprint.get_attribute("image_size_x").as_int()
    image_h = camera_blueprint.get_attribute("image_size_y").as_int()
    fov = camera_blueprint.get_attribute("fov").as_float()

    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

    # Intinsic parameters
    # K matrices - same for all 4 cameras

    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal  # In this case Fx and Fy are the same since the pixel aspect

    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0

    return K


def get_projection_matrix(K_matrix, offset=0.0):
    #Projection matrix is calculated according to the following formula.
    #P(i)rect = [[fu 0  cx - fu * bx],
    #           [0  fv  cy - fv * by],
    #           [0   0   1  0]]
    # bx is the side offset related to camera 0
    # by is the front/back offset related to camera 0. This parameter is 0 in the KITTI setup,
    # because all 4 cameras are perfectly aligned here.

    #We need a 3x4 Matrix
    add_column = np.zeros((3, 1))
    projection_matrix = np.hstack((K_matrix, add_column))
    projection_matrix[0, 3] = - projection_matrix[0, 0] * offset

    return projection_matrix


def calculate_transformation_matrix(source_carla_sensor_transform, target_carla_sensor_transform):
    # transformation matrix to convert from left-(UE-Engine) to right hand coordinate system
    # z_up_2_y_up = np.zeros([4, 4])
    # z_up_2_y_up[2, 0] = 1.0
    # z_up_2_y_up[0, 1] = -1.0
    # z_up_2_y_up[1, 2] = -1.0
    # z_up_2_y_up[3, 3] = 1.0

    TbI = np.array(target_carla_sensor_transform.get_inverse_matrix())
    Ta = np.array(source_carla_sensor_transform.get_matrix())

    source_2_target = np.dot(TbI, Ta)

    # switch axisl

    return source_2_target

    # lidar_2_world = source_carla_sensor_transform.get_matrix()
    # world_2_camera = cam02_pose.get_inverse_matrix()
    # lidar_2_camera = np.dot(world_2_camera, lidar_2_world)
    # lidar_2_image = np.dot(camera_2_image, lidar_2_camera)
    # Tr_velo_to_cam = lidar_2_image

    return Tr_velo_to_cam


def convert_2_y_up_coord_system(transformation_matrix_numpy):
    """
    Transformation matrix to convert from left-(UE-Engine) to right hand coordinate system
    """

    if transformation_matrix_numpy.shape != (4, 4):
        if transformation_matrix_numpy.shape == (3, 4):
            homogenous_part = np.array([[0, 0, 0, 1]])
            transformation_matrix_numpy = np.concatenate((transformation_matrix_numpy, homogenous_part))
            logging.debug("Input matrix in wrong shape: " + str(transformation_matrix_numpy.shape) +
                          ". Added homogenous part to adapt it to 4x4")
        else:
            raise ValueError(
                "Input matrix does not have the required shape: 4x4. Got: " + str(transformation_matrix_numpy.shape))

    z_up_2_y_up = np.zeros([4, 4])
    z_up_2_y_up[2, 0] = 1.0
    z_up_2_y_up[0, 1] = -1.0
    z_up_2_y_up[1, 2] = -1.0
    z_up_2_y_up[3, 3] = 1.0

    transformation_matrix_numpy_switched = np.dot(z_up_2_y_up, transformation_matrix_numpy)

    return transformation_matrix_numpy_switched


# def draw_3d_bounding_boxes(display, bounding_boxes):
#     """
#     Draws bounding boxes on pygame display.
#     """

#     bb_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
#     bb_surface.set_colorkey((0, 0, 0))
#     for bbox in bounding_boxes:
#         points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
#         # draw lines
#         # base
#         pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
#         pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
#         pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
#         pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
#         pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
#         # top
#         pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
#         pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
#         pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
#         pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
#         # base-top
#         pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
#         pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
#         pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
#         pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
#     display.blit(bb_surface, (0, 0))

# def draw_2d_bounding_boxes(display, boxes_2d):
#     for bbox_2d in boxes_2d:
#         (min_x, min_y, max_x, max_y) = bbox_2d
#         bbox_2d_width = (max_x - min_x)
#         bbox_2d_height = (max_y - min_y)
#         rect = pygame.Rect((int(min_x), int(min_y)), (int(bbox_2d_width), int(bbox_2d_height)))
#         pygame.draw.rect(display, (255, 0, 255), rect, 1)

def calc_projected_2d_bbox_from3d(vertices_pos3d):
    """ Takes in all vertices in pixel projection and calculates min and max of all x and y coordinates.
        Returns left top, right bottom pixel coordinates for the 2d bounding box as a list of four values.
        Note that vertices_pos2d contains a list of (y_pos2d, x_pos2d) tuples, or None
    """
    x_min, x_max = 10000, -10000
    y_min, y_max = 10000, -10000
    
    edges = [[0,1], [1,2], [3,2], [3,0], [0,4], [4,5], [5,1], [5,6], [7,6], [6,2], [7,3], [4,7]]

    for edge in edges:
        p1_ = vertices_pos3d[edge[0], :-1]
        p2_ = vertices_pos3d[edge[1], :-1]
        p1 = np.array([p1_[0,0], p1_[0,1]])
        p2 = np.array([p2_[0,0], p2_[0,1]])

        p1_in_canvas = point_in_canvas2(p1)
        p2_in_canvas = point_in_canvas2(p2)

        # Both points are out of the canvas
        if not p1_in_canvas and not p2_in_canvas:
            continue    

        # Draw 2D Bounding Boxes
        p1_temp, p2_temp = (p1.copy(), p2.copy())

        # One of the point is out of the canvas
        if not (p1_in_canvas and p2_in_canvas):
            p = [0, 0]

            # Find the intersection of the edge with the window border
            p_in_canvas, p_not_in_canvas = (p1, p2) if p1_in_canvas else (p2, p1)
            k = (p_not_in_canvas[1] - p_in_canvas[1]) 
            if p_not_in_canvas[0] - p_in_canvas[0] != 0.: 
                k /= (p_not_in_canvas[0] - p_in_canvas[0])
            else:
                k /= 1.e-10 

            x = np.clip(p_not_in_canvas[0], 0, WINDOW_WIDTH)
            y = k * (x - p_in_canvas[0]) + p_in_canvas[1]

            if y >= WINDOW_HEIGHT:
                p[0] = (WINDOW_HEIGHT - p_in_canvas[1]) / k + p_in_canvas[0]
                p[1] = WINDOW_HEIGHT - 1
            elif y <= 0:
                p[0] = (0 - p_in_canvas[1]) / k + p_in_canvas[0]
                p[1] = 0
            else:
                p[0] = WINDOW_WIDTH - 1 if x == WINDOW_WIDTH else 0
                p[1] = y

            p1_temp, p2_temp = (p, p_in_canvas)

        # Find the rightmost vertex
        x_max = p1_temp[0] if p1_temp[0] > x_max else x_max
        x_max = p2_temp[0] if p2_temp[0] > x_max else x_max

        # Find the leftmost vertex
        x_min = p1_temp[0] if p1_temp[0] < x_min else x_min
        x_min = p2_temp[0] if p2_temp[0] < x_min else x_min

        # Find the highest vertex
        y_max = p1_temp[1] if p1_temp[1] > y_max else y_max
        y_max = p2_temp[1] if p2_temp[1] > y_max else y_max

        # Find the lowest vertex
        y_min = p1_temp[1] if p1_temp[1] < y_min else y_min
        y_min = p2_temp[1] if p2_temp[1] < y_min else y_min

    return [x_min, y_min, x_max, y_max]

def calc_projected_2d_bbox(vertices_pos2d):
    """ Takes in all vertices in pixel projection and calculates min and max of all x and y coordinates.
        Returns left top, right bottom pixel coordinates for the 2d bounding box as a list of four values.
        Note that vertices_pos2d contains a list of (y_pos2d, x_pos2d) tuples, or None
    """
    x_coords = vertices_pos2d[:, 0]
    y_coords = vertices_pos2d[:, 1]
    min_x, max_x = np.min(x_coords), np.max(x_coords)
    min_y, max_y = np.min(y_coords), np.max(y_coords)
    return [min_x, min_y, max_x, max_y]


def proj_to_camera(pos_vector, extrinsic_mat):
    # transform the points to camera
    transformed_3d_pos = np.dot(inv(extrinsic_mat), pos_vector)
    return transformed_3d_pos


def crop_boxes_in_canvas(cam_bboxes):
    neg_x_inds = np.where(cam_bboxes[:, 0] < 0)[0]
    out_x_inds = np.where(cam_bboxes[:, 0] > WINDOW_WIDTH)[0]
    neg_y_inds = np.where(cam_bboxes[:, 1] < 0)[0]
    out_y_inds = np.where(cam_bboxes[:, 1] > WINDOW_HEIGHT)[0]
    cam_bboxes[neg_x_inds, 0] = 0
    cam_bboxes[out_x_inds, 0] = WINDOW_WIDTH
    cam_bboxes[neg_y_inds, 1] = 0
    cam_bboxes[out_y_inds, 1] = WINDOW_HEIGHT


def point_in_canvas(pos):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < WINDOW_HEIGHT) and (pos[1] >= 0) and (pos[1] < WINDOW_WIDTH):
        return True
    return False

def point_in_canvas2(pos, img_h=WINDOW_HEIGHT, img_w=WINDOW_WIDTH):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < img_w) and (pos[1] >= 0) and (pos[1] < img_h):
        return True
    return False


# TODO Use Carla API or pygame for drawing a rectangle
def draw_rect(array, pos, size, color=(255, 0, 0)):
    """Draws a rect"""
    point_0 = (pos[0] - size / 2, pos[1] - size / 2)
    point_1 = (pos[0] + size / 2, pos[1] + size / 2)
    if point_in_canvas(point_0) and point_in_canvas(point_1):
        for i in range(size):
            for j in range(size):
                array[int(point_0[0] + i), int(point_0[1] + j)] = color


def point_is_occluded(point, vertex_depth, depth_map):
    """ Checks whether or not the four pixels directly around the given point has less depth than the given vertex depth
        If True, this means that the point is occluded.
    """
    y, x = map(int, point)
    from itertools import product
    neigbours = product((1, -1), repeat=2)
    is_occluded = []
    for dy, dx in neigbours:
        if point_in_canvas((dy + y, dx + x)):
            # If the depth map says the pixel is closer to the camera than the actual vertex
            if depth_map[y + dy, x + dx] < vertex_depth:
                is_occluded.append(True)
            else:
                is_occluded.append(False)
    # Only say point is occluded if all four neighbours are closer to camera than vertex
    return all(is_occluded)
