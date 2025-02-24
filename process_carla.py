import copy
import os
import random
from matplotlib import pyplot as plt
import numpy as np
import cv2
import open3d as o3d

def rgb_to_nir_simulation(image):
    # Define NIR weights for each channel (R, G, B)
    nir_weights = [0.333, 0.333, 0.333]
    
    # Split the image into R, G, B channels
    b, g, r = cv2.split(image)
    
    # Apply the weights to each channel
    nir_image = (nir_weights[0] * r + nir_weights[1] * g + nir_weights[2] * b).astype(np.uint8)
    
    return nir_image

def simulate_intensity(pc_reference, normals, box_masks, ingrey_colors, mask_img, middle_points):

    retro_car_mask = ingrey_colors > 0.7
    retro_env_mask = ingrey_colors > 0.9
    random_ambient = np.random.uniform(0, 0.10, 1)


    grey_colors = ingrey_colors*0.4+0.20+random_ambient
    pc_intensity = copy.deepcopy(pc_reference)

    #calculate elevation for each point
    points = pc_intensity[:, :3]
    elevation = np.arcsin(points[:, 2]/np.linalg.norm(points, axis=1))
    max_el = np.max(elevation)
    min_el = np.min(elevation)
    #get coordinate from 0 to 64
    elevation = np.floor((elevation - min_el) / (max_el - min_el) * 63.99).astype(np.int32)
    #get 64 random factors gaussian distributed between 0.8 and 1.2
    random_factors = np.abs(np.random.normal(1, 0.15, 64))

    random.seed(0)

    random_shiny_global =  np.abs(np.random.normal(0, 0.15, 1))
    shiny = np.ones(pc_intensity.shape[0])*random_shiny_global
    #grey_colors[~mask_img] = np.mean(grey_colors[mask_img])
    drop_mask = np.zeros(pc_intensity.shape[0], dtype=bool)
    car_mask = np.zeros(pc_intensity.shape[0], dtype=bool)
    plate_mask = np.zeros(pc_intensity.shape[0], dtype=bool)



    for box_mask, middle_point in zip(box_masks, middle_points):

        #box_points = np.array(box.get_box_points())
        #mask = check_if_point_in_box(pc_intensity, box_points[:, :3], thresholds=(0,0.12,0))
        #inner_box_points = np.array(inner_box.get_box_points())
        #inner_mask = check_if_point_in_box(pc_intensity, inner_box_points[:, :3],thresholds=(0,0,0))
        #mask = mask #& ~inner_mask
        mask = box_mask

        front_middle = middle_point[0]  
        back_middle = middle_point[1]

        pc_box = pc_intensity[mask]
        #add points to mask that have at most 50 cm distance to the middle points
        
       
        pmask = np.linalg.norm(pc_box[:, :3] - front_middle, axis=1) < 0.75
        pmask = pmask | (np.linalg.norm(pc_box[:, :3] - back_middle, axis=1) < 0.75)
        plate_mask[mask] = pmask
        #print mean dists and mask pmask sum
        #print(np.mean(np.linalg.norm(pc_box[:, :3] - front_middle, axis=1)), np.mean(np.linalg.norm(pc_box[:, :3] - back_middle, axis=1)), pmask.sum())

        car_mask = car_mask | mask

     
 
        ratio = np.random.uniform(0.0, 1.0, 1)
        mix_col = ratio*ingrey_colors[mask]+ (1-ratio)
        brightness = np.abs(np.random.normal(0.15, 0.2, 1))
        #clip at 0.5
        brightness =np.clip(brightness, 0.08, 0.5)

        grey_colors[mask] = brightness*mix_col

        random_shiny = np.abs(np.random.normal(0.1, 1.2, 1))
        #clip shiny between 0 and 0.9
        random_shiny = np.clip(random_shiny, 0, 3)
        shiny[mask] =  random_shiny
        #print random shiny, and random col
        #print("shiny, col", random_shiny, brightness)

    retro_car_mask =  car_mask & retro_car_mask
    retro_env_mask = ~car_mask & retro_env_mask

 

    #compute view direction for each point by subtracting the point from the camera position 0,0,0
    view_directions = - pc_intensity[:, :3]
    distances = np.linalg.norm(view_directions, axis=1)    #normalize view directions
    normals = normals / np.linalg.norm(normals, axis=1).reshape(-1, 1)
    view_directions = view_directions / distances.reshape(-1, 1)
    intensities = np.sum(view_directions * normals, axis=1)
    #spec random factor between 0.0 and 0.875
    spec_random = np.random.uniform(0.0, 1)
    
    intensities = grey_colors* np.power(intensities, shiny)+ np.power(intensities, 500000) *spec_random
    intensities[car_mask]-=0.01
    
    #set to zero

    intensities[retro_env_mask] = 0.85
    intensities[plate_mask & retro_car_mask] =1


    if True:
        intensities *= (1-(distances/120))
        intensities += np.random.normal(0, 0.02, intensities.shape)
        intensities *=random_factors[elevation]

 
    rand_decider = np.random.uniform(-0.005,-0.02, drop_mask.shape)#-0.022
    drop_mask =  (intensities <0)# rand_decider)


    #clip intensities

    #print("intensities", np.min(intensities), np.max(intensities), np.mean(intensities), np.sum(drop_mask))
   
    intensities = np.clip(intensities, 0, 1)
    pc_intensity = np.hstack((pc_intensity, intensities.reshape(-1, 1)))

    #boxes+= inner_boxes

    #write_image(pc_intensity, rgb, transforms, ending=out_path + file.replace('.png', '_intensity.png'), write=True)
    #write_cloud(pc_intensity, file, velodyne_intensity_path, intensities=intensities, use_intensity=True)
    return pc_intensity, intensities, drop_mask

def check_between_planes(points, plane_a, plane_b, threshold=2):
    #point is between planes if dot products between planes have different signs, point_a is point on plane_a, point_b is point on plane_b

    #threshold for dot product

    point_a, normal_a = plane_a
    point_b, normal_b = plane_b

    # Ensure points are in a numpy array
    points = np.asarray(points)
    
    # Compute the vectors from the points on the planes to the points
    vec_points_a = points - point_a
    vec_points_b = points - point_b

    # Compute the dot products
    dot_a = np.dot(vec_points_a, normal_a)
    dot_b = np.dot(vec_points_b, normal_b)

    # Apply the threshold
    dot_a = np.where(np.abs(dot_a) <= threshold, 0, dot_a)
    dot_b = np.where(np.abs(dot_b) <= threshold, 0, dot_b)

    # Check if the signs of the dot products are different
    signs_a = np.sign(dot_a)
    signs_b = np.sign(dot_b)

    # Points are between planes if signs are different and neither is zero
    mask = (signs_a != signs_b) & (signs_a != 0) & (signs_b != 0)

    return ~mask


def compute_planes_from_corners(base_corner,left_corner, top_corner, opposite_corner) :


    #front normal
    #front normal is the normal of the plane that is perpendicular to the front face of the box
    #get the edge vectors
    edge1 = left_corner - base_corner
    edge2 =  top_corner- base_corner
    normal_a = np.cross(edge1, edge2)
    #get a point on the plane
    point_a =base_corner

    #back normal
    normal_b = -normal_a
    point_b = opposite_corner



    return [(point_a, normal_a), (point_b, normal_b)]
   

def lidar_effects(point_cloud, outer_boxes, inner_boxes):

    

    for i in range(len(outer_boxes)):
        ob= outer_boxes[i]
        ib = inner_boxes[i]

        ob_max_bounds = ob.get_max_bound()
        ob_min_bounds = ob.get_min_bound()

        ib_max_bounds = ib.get_max_bound()
        ib_min_bounds = ib.get_min_bound()
     
        #get points in outer_box
        mask = check_if_point_in_box(point_cloud, np.asarray(ob.get_box_points())[:, :3], threshold=0)
        points = point_cloud[mask]
        point_cloud = point_cloud[~mask]
  
        num_points = points.shape[0]
        #remove points from pointcloud

        #num_points is square root of number of points 
        num_points = int(np.sqrt(num_points))*30 + np.random.randint(0, 10)
  

        #generate 4 new points in the outer box 
        if True:
            point_cloud_box = points
            #num spheres = random between 0 and 20
            num_spheres = np.random.randint(2, 32)
            for j in range(num_spheres):
                #get random point in outer box
                x = np.random.uniform(ob_min_bounds[0], ob_max_bounds[0])
                y = np.random.uniform(ob_min_bounds[1], ob_max_bounds[1])
                z = np.random.uniform(ob_min_bounds[2], ob_max_bounds[2])


                #compute distances to all points in the outer box
                distances = np.linalg.norm(point_cloud_box[:,:3] - np.array([x, y, z]), axis=1)
                #remove points randomly dpeneding on distance

                rnd_decider = np.random.uniform(0.4, .65, distances.shape[0])
                mask = distances > rnd_decider
                point_cloud_box = point_cloud_box[mask]

            point_cloud = np.vstack((point_cloud, point_cloud_box))

        if(num_points >50):
            half_z =  (ib_max_bounds[2] + ib_min_bounds[2]) / 2
            for j in range(4):
                #get random point in inner box
                x = np.random.uniform(ib_min_bounds[0], ib_max_bounds[0])
                y = np.random.uniform(ib_min_bounds[1], ib_max_bounds[1])
                z = np.random.uniform(half_z, ib_max_bounds[2])

                #sample between 0.05 and 0.1 * num_points new points that are gaussian distributed around the random point
                # get random number of points
                num_points_gen = np.random.randint(int(0.001*num_points), int(0.02*num_points)) 
                distr = np.random.uniform(0.1, 0.15)
                new_points = np.random.normal([x, y, z], distr, (num_points_gen, 3))
                #add zero intensity to new points
                new_points = np.hstack((new_points, np.zeros((num_points_gen, 1))))
                point_cloud = np.vstack((point_cloud, new_points))


    return point_cloud

def shrink_box_length(rot_y, t, pc, mask, transforms, box_points):
    p_rect_02, R0_rect, Tr_velo_to_cam = transforms


    pc = pc[mask]

    pc = pc[:, :3]
    pc = np.hstack((pc, np.ones((pc.shape[0], 1))))
    pc = np.dot(Tr_velo_to_cam, pc.T).T
    pc = np.dot((R0_rect), pc.T).T



    box_points = np.dot(Tr_velo_to_cam, box_points.T).T
    box_points = np.dot((R0_rect), box_points.T).T
    box_points = box_points[:, :3]
    box_points = box_points - t


    #divide by w
   # pc = pc[:, :3] / pc[:, 3].reshape(-1, 1)
    points = pc[:, :3]

    points = points - t
    R = np.array([[np.cos(-rot_y), 0, np.sin(-rot_y)],
                    [0, 1, 0],
                    [-np.sin(-rot_y), 0, np.cos(-rot_y)]])

    points = np.dot(R, points.T).T

    box_points= np.dot(R, box_points.T).T


    #min_length_factor is dependend on max dist of box points


    #get axis aligned box
    max_x = np.max(points[:, 0])
    min_x = np.min(points[:, 0])
    max_y = np.max(points[:, 1])
    min_y = np.min(points[:, 1])
    max_z = np.max(points[:, 2])
    min_z = np.min(points[:, 2])



    
    #length
    delta_max = np.abs(np.max(box_points[:, 0]) - max_x)
    delta_min = np.abs(min_x - np.min(box_points[:, 0]))
    threshold_max = 0.25*delta_max+0.2
    threshold_min = 0.25*delta_min+0.2
    max_x = np.clip(max_x, np.max(box_points[:, 0])-threshold_max, np.max(box_points[:, 0]))
    min_x = np.clip(min_x, np.min(box_points[:, 0]), np.min(box_points[:, 0]+threshold_min))

    #height: 
    #bot

    delta_max = np.abs(np.max(box_points[:, 1]) - max_y)
    threshold_max = 0.05*delta_max+0.05
    max_y = np.clip(max_y, np.max(box_points[:, 1])-threshold_max, np.max(box_points[:, 1])) 
    #top
    min_y =   np.clip(min_y, np.min(box_points[:, 1]), np.min(box_points[:, 1])+0.1)-0.025


    #width
    delta_max = np.abs(np.max(box_points[:, 2]) - max_z)
    delta_min = np.abs(min_z - np.min(box_points[:, 2]))
    threshold_max = 0.25*delta_max+0.05
    threshold_min =0.25*delta_min+0.05

    max_z = np.clip(max_z, np.max(box_points[:, 2])-threshold_max, np.max(box_points[:, 2]))
    min_z = np.clip(min_z, np.min(box_points[:, 2]), np.min(box_points[:, 2]+threshold_min))
    



    max_point = np.array([max_x, max_y, max_z])
    min_point = np.array([min_x, min_y, min_z])

    h_new = max_point[1] - min_point[1]
    w_new = max_point[2] - min_point[2]
    l_new = max_point[0] - min_point[0]


    #undo rotation
    R = np.array([[np.cos(rot_y), 0, np.sin(rot_y)],
                    [0, 1, 0],
                    [-np.sin(rot_y), 0, np.cos(rot_y)]])
    
    max_point = np.dot(R, max_point)
    min_point = np.dot(R, min_point)

    #undo translation
    max_point = max_point + t
    min_point = min_point + t

    center= (max_point + min_point) / 2




    center[1] += h_new/2


    extents = h_new, w_new, l_new


    return center, extents
    

def shrink_box(rot_y, t, pc, mask, transforms, box_points):


    p_rect_02, R0_rect, Tr_velo_to_cam = transforms
  
    pc = pc[mask]



    pc = pc[:, :3]
    pc = np.hstack((pc, np.ones((pc.shape[0], 1))))
    pc = np.dot(Tr_velo_to_cam, pc.T).T
    pc = np.dot((R0_rect), pc.T).T

    box_points = np.hstack((box_points, np.ones((box_points.shape[0], 1))))
    box_points = np.dot(Tr_velo_to_cam, box_points.T).T
    box_points = np.dot((R0_rect), box_points.T).T
    box_points = box_points[:, :3]
    box_points = box_points - t


    #divide by w
   # pc = pc[:, :3] / pc[:, 3].reshape(-1, 1)
    points = pc[:, :3]

    points = points - t
    R = np.array([[np.cos(-rot_y), 0, np.sin(-rot_y)],
                    [0, 1, 0],
                    [-np.sin(-rot_y), 0, np.cos(-rot_y)]])

    points = np.dot(R, points.T).T

    box_points= np.dot(R, box_points.T).T


    #if at least 5 points

    if points.shape[0] > 4:
        y = points[:, 1]

        fierce_threshold = np.percentile(y, 99)

        fierce_mask = y < (fierce_threshold-0.35)

    
        #threshold = np.percentile(y, 90)
        #low_mask = y > threshold
        #biggest_y = np.max(y)+0.05
        #points[low_mask] = (0, biggest_y, 0)

       # print(fierce_mask.sum(), points.shape[0])
        if fierce_mask.sum() > 0.6 * points.shape[0]:
            points = points[fierce_mask]

    #get axis aligned box
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])
    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])



    min_x_old = np.min(box_points[:, 0])
    max_x_old = np.max(box_points[:, 0])
    min_y_old = np.min(box_points[:, 1])
    max_y_old = np.max(box_points[:, 1])
    min_z_old = np.min(box_points[:, 2])
    max_z_old = np.max(box_points[:, 2])

    extents = np.array([max_x_old - min_x_old, max_y_old - min_y_old, max_z_old - min_z_old])


    threshold_change = 0.01
    increase_factor = 15

    min_x = np.clip(min_x, min_x_old - threshold_change*extents[0]*increase_factor, min_x_old + threshold_change*extents[0])
    max_x = np.clip(max_x, max_x_old - threshold_change*extents[0], max_x_old + threshold_change*extents[0]*increase_factor)
    min_y = np.clip(min_y, min_y_old - threshold_change*extents[1]*increase_factor, min_y_old + threshold_change*extents[1])
    max_y = np.clip(max_y, max_y_old - threshold_change*extents[1], max_y_old + threshold_change*extents[1]*increase_factor)
    min_z = np.clip(min_z, min_z_old - threshold_change*extents[2]*increase_factor, min_z_old + threshold_change*extents[2])
    max_z = np.clip(max_z, max_z_old - threshold_change*extents[2], max_z_old + threshold_change*extents[2]*increase_factor)
    

    #if delta between old and new is bigger than threshold * increase factor use old value
    increase_factor*=1.01
    if np.abs(min_x - min_x_old) > threshold_change*extents[0]*increase_factor:
        min_x = min_x_old
    if np.abs(max_x - max_x_old) > threshold_change*extents[0]*increase_factor:
        max_x = max_x_old
    if np.abs(min_y - min_y_old) > threshold_change*extents[1]*increase_factor:
        min_y = min_y_old
    if np.abs(max_y - max_y_old) > threshold_change*extents[1]*increase_factor:
        max_y = max_y_old
    if np.abs(min_z - min_z_old) > threshold_change*extents[2]*increase_factor:
        min_z = min_z_old
    if np.abs(max_z - max_z_old) > threshold_change*extents[2]*increase_factor:
        max_z = max_z_old
    if min_x < min_x_old - threshold_change*extents[0]*increase_factor:
        min_x = min_x_old

    max_point = np.array([max_x, max_y, max_z])
    min_point = np.array([min_x, min_y, min_z])

    l_new = max_x - min_x +0.001
    h_new = max_y - min_y +0.001
    w_new = max_z - min_z +0.001


    #undo rotation
    R = np.array([[np.cos(rot_y), 0, np.sin(rot_y)],
                    [0, 1, 0],
                    [-np.sin(rot_y), 0, np.cos(rot_y)]])
    
    max_point = np.dot(R, max_point)
    min_point = np.dot(R, min_point)

    #undo translation
    max_point = max_point + t
    min_point = min_point + t

    max_x = max_point[0]
    max_y = max_point[1]
    max_z = max_point[2]

    min_x = min_point[0]
    min_y = min_point[1]
    min_z = min_point[2]

    box_points = np.array([[max_x, max_y, max_z],
                        [max_x, max_y, min_z],
                        [max_x, min_y, max_z],
                        [max_x, min_y, min_z],
                        [min_x, max_y, max_z],
                        [min_x, max_y, min_z],
                        [min_x, min_y, max_z],
                        [min_x, min_y, min_z]])



    center = (max_point + min_point) / 2
    #add h/2 to y
    center[1] += h_new/2

    

    return center, w_new, h_new, l_new



def check_if_point_in_box(points, box_points, thresholds=(2,2,2)):

    corners = box_points

    #move corners upwards by 0.1

 
    #only use first 3 dimensions
    points = points[:, :3]
    mask = np.ones(points.shape[0], dtype=bool)

    #get the base corner
    base_corner = corners[0]
    #get the left corner
    left_corner = corners[1]
    #get the top corner
    top_corner = corners[3]
    #get the opposite corner
    opposite_corner = corners[2]
    planes = compute_planes_from_corners(base_corner, left_corner, top_corner, opposite_corner)
    #check if point is between the two planes
    mask = check_between_planes(points, planes[0], planes[1], threshold=thresholds[0])





    left_corner = corners[2]
    opposite_corner = corners[1]
    planes = compute_planes_from_corners(base_corner, left_corner, top_corner, opposite_corner)
    mask = np.logical_and(mask, check_between_planes(points, planes[0], planes[1], threshold=thresholds[1]))



    top_corner = corners[1]
    opposite_corner = corners[4]
    planes = compute_planes_from_corners(base_corner, left_corner, top_corner, opposite_corner)

    mask = np.logical_and(mask, check_between_planes(points, planes[0], planes[1], threshold=thresholds[1]))




    return mask


def write_label(old_label_path, new_label_path, new_label_values, og = False):
    labels = np.loadtxt(old_label_path, dtype=str, delimiter=" ")
    if len(labels.shape) == 1:
        labels = [labels]
    new_labels = []
    factors = 0
    f_c = 0
    obj_types = []
    for i, label in enumerate(labels):
        if len(new_label_values[i]) == 0:
            continue
        roty, center, dimensions = new_label_values[i]
        #print(roty, center, dimensions)#
        new_label = label.copy()
        volume_before = float(label[8]) * float(label[9]) * float(label[10])
        obj_type = new_label[0]
        obj_types.append(obj_type)
        new_label[0] = 0
        if not og:
            #remove type

            #convert to float
            new_label = new_label.astype(np.float64)

            new_label[14] = roty
            new_label[11] = center[0]

            new_label[12] = center[1]
            new_label[13] = center[2]
            new_label[8] = dimensions[1]#h
            new_label[9] = dimensions[0]#w
            new_label[10] = dimensions[2]#l

        volume = float(new_label[8] ) * float(new_label[9]) * float(new_label[10])
        factor = volume/volume_before
        factors += factor
        f_c += 1
        #add type
        new_labels.append(new_label)


    print("number of new labels", len(new_labels), len(labels))
    if len(new_labels) > 0:
        print("average factor", factors/f_c)
    print(new_label_path)

    #for all new labels write type and label in the same line

    with open(new_label_path, 'w') as f:
        for label, obj_type in zip(new_labels, obj_types):
            #remove first element
            label = label[1:]
            f.write(obj_type + " " + " ".join(label.astype(str)) + "\n")


    

def read_label(label_path, file, shrink = False):
    #if path does not exist return empty list


    label_file = label_path + file[:-4] + ".txt"
    if not os.path.exists(label_file):
        return [], [], [], [], [], []
    labels = np.loadtxt(label_path + file[:-4] + ".txt", dtype=str, delimiter=" ")
    #print(label_path + file[:-4] + ".txt")
    boxes = []
    rotys = []
    positions = []
    whl= []
    types = []
    scores = []
    #if only one label make it a list
    if len(labels.shape) == 1:
        labels = [labels]

    for label in labels:

        #if label[0] == "Car":
        #    #print(label)
       
        rot_y = float(label[14])
        types.append(label[0])

        
        h = float(label[8])
        w = float(label[9])
        l = float(label[10])
        #get t from location
        t = np.array([float(label[11]), float(label[12])-0.04, float(label[13])])
        label[12] = str(t[1])


        box = gen_box(w, h, l, t, rot_y)
        boxes.append(box)
        rotys.append(rot_y)
        positions.append(t)
        whl.append([w, h, l])

        #score is label[15] if not exists score is 0
        if len(label) > 15:
            score = float(label[15])
        else:
            score = 0

        scores.append(score)

    return boxes, rotys, positions, whl, types, scores, labels

def gen_box(w,h,l, t, rot_y):

    # compute rotational matrix around yaw axis
    R = np.array([[np.cos(rot_y), 0, np.sin(rot_y)],
                    [0, 1, 0],
                    [-np.sin(rot_y), 0, np.cos(rot_y)]])

    # 3d bounding box corners
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
 

    box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(corners_3d.T))
    #translate box
    box.translate(t)
    #color green
    box.color = (0, 1, 0)
    return box


def write_image(point_cloud, rgb, transforms, ending = "out", write = False):
     
    #remove points with z <= 0

    p_rect_02, R0_rect, Tr_velo_to_cam = transforms


    #if point cloud shape is 4 remove last column
    intensity = None
    if point_cloud.shape[1] == 4:
        intensity = point_cloud[:, 3]
        point_cloud = point_cloud[:, :3]


    point_cloud = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
    cam = p_rect_02 @ R0_rect @ Tr_velo_to_cam @ point_cloud.T

    #remove points with z <= 0
    z_mask = cam[2, :] > 0
 
    
    
    cam = cam[:, z_mask]

    #get u,v,z
    cam[:2] /= cam[2,:]
    cam = cam.T

    rgb_w = rgb.shape[1]
    rgb_h = rgb.shape[0]
    #remove points outside image
    mask = (cam[:, 0] > 0) & (cam[:, 0] < rgb_w) & (cam[:, 1] > 0) & (cam[:, 1] < rgb_h)
    cam = cam[mask]

    z_mask[z_mask] =  mask

    u, v, z = cam.T
    u = u.astype(int)
    v = v.astype(int)


    
    #rgb_colors will be new dimension of point cloud so i need 3 channels x length of point cloud
    rgb_colors = np.zeros((point_cloud.shape[0], 1))
    #fill rgb_colors with rgb values
    rgb_colors = rgb[v, u]/255.0

    if write:
        #print(ending)
        cv2.imwrite(ending, rgb)

    return rgb, z_mask, rgb_colors

def box_from_corners(corners):
    """
    Convert 8 corner points of a 3D box into center, axes, and half-lengths.
    
    :param corners: List or array of 8 corner points, each in the form (x, y, z).
    :return: (center, axes, half_lengths)
    """
    # Convert to numpy array for easier manipulation
    corners = np.array(corners)

    # Calculate the center (mean of all corner points)
    center = np.mean(corners, axis=0)
    
    # Calculate axes by subtracting the center from each corner
    vectors = corners - center

    # Get the axes as the directions of the box
    axis1 = vectors[0] - vectors[1]  # Vector along x-axis
    axis2 = vectors[0] - vectors[2]  # Vector along y-axis
    axis3 = vectors[0] - vectors[4]  # Vector along z-axis

    # Normalize the axes
    axis1 = axis1 / np.linalg.norm(axis1)
    axis2 = axis2 / np.linalg.norm(axis2)
    axis3 = axis3 / np.linalg.norm(axis3)

    # Calculate the half lengths of the box along each axis
    half_lengths1 = np.max(np.abs(np.dot(vectors, axis1))) / 2
    half_lengths2 = np.max(np.abs(np.dot(vectors, axis2))) / 2
    half_lengths3 = np.max(np.abs(np.dot(vectors, axis3))) / 2

    return center, [axis1, axis2, axis3], [half_lengths1, half_lengths2, half_lengths3]

def is_overlap_obb_from_corners(corners1, corners2):
    """
    Check if two 3D oriented bounding boxes (OBBs), represented by corner points, overlap using SAT.
    
    :param corners1: 8 corner points of the first box
    :param corners2: 8 corner points of the second box
    :return: True if the boxes overlap, False otherwise.
    """
    # Get the center, axes, and half-lengths from the corner points
    center1, axes1, half_lengths1 = box_from_corners(corners1)
    center2, axes2, half_lengths2 = box_from_corners(corners2)

    # Vector from center1 to center2
    T = center2 - center1

    # Check each of the 15 potential separating axes
    for i in range(3):
        # Test axes from box1
        if not overlap_on_axis(T, axes1[i], axes1, axes2, half_lengths1, half_lengths2):
            return False
        # Test axes from box2
        if not overlap_on_axis(T, axes2[i], axes1, axes2, half_lengths1, half_lengths2):
            return False
    
    # Test cross products of axes1[i] with axes2[j]
    for i in range(3):
        for j in range(3):
            axis = np.cross(axes1[i], axes2[j])
            if not np.allclose(axis, 0):  # Avoid degenerate cross products (parallel axes)
                if not overlap_on_axis(T, axis, axes1, axes2, half_lengths1, half_lengths2):
                    return False

    return True  # Overlap found on all axes

def overlap_on_axis(T, axis, axes1, axes2, half_lengths1, half_lengths2):
    """
    Check if projections of two boxes overlap on a given axis.
    
    :param T: Vector between box centers
    :param axis: Axis to test for overlap
    :param axes1: Axes of the first box
    :param axes2: Axes of the second box
    :param half_lengths1: Half lengths of the first box along each axis
    :param half_lengths2: Half lengths of the second box along each axis
    :return: True if there is overlap on the axis, False otherwise
    """
    # Normalize the axis to get an accurate projection
    axis = axis / np.linalg.norm(axis)
    
    # Project half-lengths of both boxes onto the axis
    projection1 = sum(abs(np.dot(axis, a)) * h for a, h in zip(axes1, half_lengths1))
    projection2 = sum(abs(np.dot(axis, a)) * h for a, h in zip(axes2, half_lengths2))
    
    # Project the vector T onto the axis
    distance = abs(np.dot(T, axis))
    
    # Check for overlap
    return distance <= projection1 + projection2


def filter_cloud(point_cloud, change_elevation=False, min_distance=1, max_distance=100):

    mask = (np.linalg.norm(point_cloud, axis=1) > min_distance) & (np.linalg.norm(point_cloud, axis=1) < max_distance) & (point_cloud[:, 0] > 0.01)
    indices = np.arange(point_cloud.shape[0])
    point_cloud = point_cloud[mask]
    indices = indices[mask]

    #calculate azimuth and elevation for each point
    azimuth = np.arctan2(point_cloud[:, 1],  point_cloud[:, 0])

    azimuth= np.floor(azimuth / np.radians(0.16)).astype(np.int32)
    distance = np.linalg.norm(point_cloud, axis=1)
    elevation = np.arcsin(point_cloud[:,2]/distance) #np.arctan2(point_cloud[:, 0], point_cloud[:, 2])
    if change_elevation:

        #elevation in kitti should be between -24.9 and 2.0
        elevation_deg = np.degrees(elevation)
        mask = (elevation_deg > -26) & (elevation_deg < 2.2)
        indices = indices[mask]
        point_cloud = point_cloud[mask]
        elevation = elevation[mask]
        azimuth = azimuth[mask]
        elevation = np.floor(elevation / np.radians(0.44)).astype(np.int32)
    else:
        elevation = np.floor(elevation / np.radians(0.30)).astype(np.int32)

    coords = np.vstack((azimuth, elevation)).T
    unique, idx = np.unique(coords, axis=0, return_index=True)

    point_cloud = point_cloud[idx]
    indices = indices[idx]

    #jitter the point cloud
    #point_cloud += np.random.normal(0, 0.0050, point_cloud.shape)
        
    return point_cloud, indices

def carla_depth_to_cloud(depth_img_path, file):
        #depth_img[:top_crop+offset, :] = (0,0,0)
        depth_img = cv2.imread(depth_img_path + file)
        depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2RGB)
        #convert to to 32 bit float
        depth_img = depth_img.astype(np.float32)
        depth_img = depth_img[:,:,0]+depth_img[:,:,1]*256+depth_img[:,:,2]*256*256

        depth_img = depth_img/(256*256*256-1)

        depth_img = depth_img*1000

        #write out depth image for debugging
        #cv2.imwrite(out_path + file.replace('.png', '_depth.png'), depth_img)

        #print max and min depth
        #print(np.max(depth_img), np.min(depth_img))
    
        #get intrinsic matrix
        K = np.array([[focal_length, 0, image_w / 2], [0, focal_length, image_h / 2], [0, 0, 1]])

        #image_h = image_h_target
        #get the 3d point cloud by using the depth image and the intrinsic matrix
        np.random.seed(0)
        shutter_random = np.random.randint(0, 1)
        point_cloud = np.zeros((image_h, image_w, 3))
        for v in range(image_h):
            for u in range(image_w):
                #if u or v is greater than depth_img shape skip
                Z = depth_img[v, u] #+ (image_w-u -image_w/2)*0.0005*shutter_random
                #if depth is 0 skip
                if Z <= 0:
                    continue
                X = (u - K[0, 2]) * Z / K[0, 0]
                Y = (v - K[1, 2]) * Z / K[1, 1]
                point_cloud[v, u, :] = [X, Y, Z]

        #swap the x and z axis
        point_cloud = point_cloud[:, :, [2, 1, 0]]
        #swap y and z
        point_cloud = point_cloud[:, :, [0, 2, 1]]

        #multiply y by -1
        point_cloud[:, :, 2] *= -1
        #multiply z by -1
        point_cloud[:, :, 1] *= -1
        


        #flatten point cloud
        point_cloud = point_cloud.reshape(-1, 3)

        return point_cloud

def write_cloud(pc_velo, file, path, intensities=None, use_intensity=False):
    #overwrite 4th channel with 0s and add 0s if there is no 4th channel
    if pc_velo.shape[1] == 3:
        if not use_intensity:
            pc_velo = np.hstack((pc_velo, np.zeros((pc_velo.shape[0], 1))))
        else:
            pc_velo = np.hstack((pc_velo, intensities.reshape(-1, 1)))

    else:
        pc_velo[:, 3] = 0
        if use_intensity:
            pc_velo[:, 3] = intensities
    pc_velo = pc_velo.astype(np.float32)
    #print("writing to", path + file.replace('.png', '.bin'))
    pc_velo.tofile(path + file.replace('.png', '.bin'))

mp = False
def process_file(file):

        #print(rgb_path+ file, mp)
        #if max files reached break
        parent_path = rgb_path.split('/')[-3]

        rgb_img = cv2.imread(rgb_path + file)
        if 'rgb_' in file:
            file = file.replace('rgb_', '').replace('.jpg', '.png')

        bin_name = file.replace('png', 'bin')
        calib_name = file.replace('png', 'txt')

        #extract number from file name
        file_number = int(file.split('_')[-1].replace('.png', ''))

        label_file = label_path + file[:-4] + ".txt"
        #check if empty
        label = np.loadtxt(label_file, dtype=str, delimiter=" ")
        #if label is empty skip
        if len(label) == 0:
            return




        boxes, rotys , positions, whls, types, scores, labels = read_label(label_path, file, shrink=False)
        good_labels = []
        overlap = False
        for box in boxes:
            for box2 in boxes:
                if box == box2:
                    continue
                box_points = np.array(box.get_box_points())
                box2_points = np.array(box2.get_box_points())
                if is_overlap_obb_from_corners(box_points, box2_points):
                    print("overlap", file)
                    overlap = True
                    break
            if overlap:
                break
                    

        #convert rgb to rgb from gray
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        rgb2 = rgb_to_nir_simulation(rgb_img)


        
        calib_file_path = calib_path + calib_name
        
        with open(calib_file_path, 'r') as f:
                c_lines = f.readlines()
                #get rectification matrix
                R0_rect = np.array(c_lines[4].split(' ')[1:], dtype=np.float32).reshape([3, 3])
                R0_rect = np.vstack((R0_rect, np.array([0, 0, 0])))
                R0_rect = np.hstack((R0_rect, np.array([[0], [0], [0], [1]])))
                #get projection matrix
                p_rect_02 = np.array(c_lines[2].split(' ')[1:], dtype=np.float32).reshape([3, 4])
                Tr_velo_to_cam = np.array(c_lines[5].split(' ')[1:], dtype=np.float32).reshape([3, 4])
                Tr_velo_to_cam = np.vstack((Tr_velo_to_cam, np.array([0, 0, 0, 1])))


        transforms = (p_rect_02, R0_rect, Tr_velo_to_cam)
        #project the point cloud to the camera frame
        point_cloud_bin = np.fromfile(velodyne_path + bin_name, dtype=np.float32, count=-1).reshape([-1, 14])
        point_cloud_dual_bin = np.fromfile(velodyne_like_path + bin_name, dtype=np.float32, count=-1).reshape([-1, 14])

        point_cloud_windows = point_cloud_bin[:, :3]
        point_cloud_nw = point_cloud_bin[:, 7:10]
        point_cloud_dual = point_cloud_dual_bin[:, 7:10]

        #normals for nw
        normals = point_cloud_dual_bin[:, 11:14]
        #normalize normals to have length 1
        normals = normals / np.linalg.norm(normals, axis=1).reshape(-1, 1)

        pc_windows, collected_mask = filter_cloud(point_cloud_windows)
        pc_dual, collected_mask_dual = filter_cloud(point_cloud_dual, max_distance=max_distance)
        pc_velodyne, collected_mask = filter_cloud(point_cloud_nw)

        normals = normals[collected_mask_dual]

        rgb, img_mask,_ = write_image(pc_velodyne, rgb2, transforms, ending=out_path + file.replace('.png', '_nw.png') )
        pc_velodyne = pc_velodyne[img_mask]

        rgb, img_mask,_ = write_image(pc_windows, rgb2, transforms, ending=out_path + file.replace('.png', '_windows.png') )
        pc_windows = pc_windows[img_mask]



        point_cloud_depth = carla_depth_to_cloud(depth_img_path, file)
        pc_depth, collected_mask = filter_cloud(point_cloud_depth, change_elevation=True)
        rgb, img_mask,_ = write_image(pc_depth, rgb2, transforms, ending=out_path + file.replace('.png', '_depth.png') )
        pc_depth = pc_depth[img_mask]


        rgb, img_mask, grey_colors = write_image(pc_dual, rgb2, transforms, ending=out_path + file.replace('.png', '_dual.png') )
        pc_dual = pc_dual[img_mask]
        normals = normals[img_mask]
        base_col = np.zeros((pc_dual.shape[0]))
        base_col = grey_colors
            

        pc_reference = pc_dual
        box_masks = []
        middle_points = []

        for box, roty, t, whl, type, label in zip(boxes, rotys, positions, whls, types, labels):
               
            if type != "Pedestrian":
                #apply inverse velo to cam
                box_points = np.array(box.get_box_points())
                box_points = np.hstack((box_points, np.ones((box_points.shape[0], 1))))
                #apply inverse r0 rect
                box_points = np.dot(np.linalg.inv(R0_rect), box_points.T).T
                box_points = np.dot(np.linalg.inv(Tr_velo_to_cam), box_points.T).T
       

        
                #threshholds: width, height, length
                mask = check_if_point_in_box(pc_reference, box_points[:, :3], thresholds=(0, .5, .2))

                #if  ( overlap or type != "Car" and type != "Pedestrian") or mask.sum() < 5:# or t[1] > 1.8 or t[1] < 0.5:
                    #pc_reference = pc_reference[~mask]
                    #normals = normals[~mask]
                #    continue

                #shrink box
                if mask.sum() > 5:
                    center_new, extents_new = shrink_box_length(roty, t, pc_reference, mask, transforms, box_points)
                    

                    middle_front = np.mean(box_points[[0, 2, 3,5]], axis=0) 
                    middle_back = np.mean(box_points[[1,4,6,7]], axis=0) 
                    #drop homogenous coordinate
                    middle_front = middle_front[:3]
                    middle_back = middle_back[:3]

                    label[11] = f"{center_new[0]:.6f}"
                    label[12] = f"{center_new[1]:.6f}"
                    label[13] = f"{center_new[2]:.6f}"
                    label[8] = f"{extents_new[0]:.6f}"
                    label[9] = f"{extents_new[1]:.6f}"
                    label[10] = f"{extents_new[2]:.6f}"
        
                    middle_points.append((middle_front, middle_back))
                    good_labels.append(label)
                    box_masks.append(mask)





        #prepare good_labels so that it can be written with np.savetxt
        good_labels = np.array(good_labels)
        np.savetxt(label_out_path + file.replace('.png', '.txt'), good_labels, fmt='%s')
    

        if sim:
            pc_intensity, intensities, drop_mask = simulate_intensity(pc_reference, normals, box_masks, base_col, img_mask, middle_points)

            pc_drop = pc_reference[~drop_mask]

        else:
            intensities = np.zeros(pc_reference.shape[0])

        pc_dual = pc_reference

 
        #print("pc_velo shape", pc_velodyne.shape, "pc_windows shape", pc_windows.shape, "pc_dual shape", pc_dual.shape, "pc_depth shape", pc_depth.shape, "pc_intensity shape", pc_intensity.shape, "pc_drop shape", pc_drop.shape)
        
        write_cloud(pc_velodyne, file, velodyne_front_path)
        write_cloud(pc_windows, file, velodyne_opaque_path)
        write_cloud(pc_dual, file, velodyne_dual_path)
        write_cloud(pc_depth, file, velodyne_depth_path)
        write_cloud(pc_intensity, file, velodyne_intensity_path, intensities, use_intensity=True)
        write_cloud(pc_drop, file, velodyne_drop_path)

        pc_jitter = pc_reference + np.random.normal(0, 0.01, pc_reference.shape)
        write_cloud(pc_jitter, file, velodyne_jitter_path)


       






#main function
if __name__ == '__main__':

    import argparse
    import multiprocessing
    parser = argparse.ArgumentParser()
    parser.add_argument('--max_files', type=int, default=2000000)
    parser.add_argument('--max_distance', type=int, default=100)
    parser.add_argument('--max_sequences', type=int, default=1000)
    parser.add_argument('--file_id', type=int, default=-1)
    parser.add_argument('--use_multiprocessing', action='store_true')
    parser.add_argument('--simulate_intensity', action='store_true')
    parser.add_argument('--base_path', type=str, help='path to the raw carla output folder')

    
    args = parser.parse_args()
    allowed_types = ["Car", "Pedestrian", "Cyclist", "Van", "Truck", "Tram", "Misc"]
    max_distance = args.max_distance
    file_id = args.file_id



    sim= args.simulate_intensity
 
    use_multiprocessing = args.use_multiprocessing
    mp = use_multiprocessing

    identifier = base_path.split('/')[-4]

    subpaths = os.listdir(base_path)    
    #sort subpaths
    subpaths.sort()
    x = []
    y= []
    color = []
    dists= []
    k=0
    for path in subpaths:
    
        k+=1
        if k > args.max_sequences:
            break
        
        max_files = args.max_files
        path = base_path + path + '/'

        out_path = "/home/oq55olys/Projects/lidar-toolbox/carla_2/out_intensity/"
        if not os.path.exists(out_path):
            os.makedirs(out_path)

        plot_path = "/home/oq55olys/Projects/lidar-toolbox/carla_2/plot/"
        if not os.path.exists(plot_path):
            os.makedirs(plot_path)

        if 'VKITTI' in path:
            path += 'clone/'
        print(path)
        depth_img_path = path + 'velodyne_cam/'
        velodyne_path = path + 'velodyne/'
        velodyne_like_path = path + 'velodyne_like/'
        velodyne_path = path + 'velodyne/'
        label_path = path + 'label_2/'
        label_out_path = path + 'label_2_filtered/'

        if not os.path.exists(label_out_path):
            os.makedirs(label_out_path)



        #if not label path skip
        if not os.path.exists(label_path):
            print("label path does not exist: "+ label_path)
            continue


        depth_path_exists = os.path.exists(depth_img_path)

        #make velodyne paths below if they do not exist
        velodyne_front_path = path + 'velodyne_front/'
        velodyne_depth_path = path + 'velodyne_depth/'
        velodyne_intensity_path = path + 'velodyne_intensity/'
        velodyne_dual_path = path + 'velodyne_dual/'
        velodyne_opaque_path = path + 'velodyne_opaque/'
        velodyne_drop_path = path + 'velodyne_drop/'
        velodyne_jitter_path = path + 'velodyne_jitter/'
        #velodyne_combined_path = path + 'velodyne_combined/'


        if not os.path.exists(velodyne_front_path):
            os.makedirs(velodyne_front_path)
        if not os.path.exists(velodyne_intensity_path):
            os.makedirs(velodyne_intensity_path)
        if not os.path.exists(velodyne_dual_path):
            os.makedirs(velodyne_dual_path)
        if not os.path.exists(velodyne_opaque_path):
            os.makedirs(velodyne_opaque_path)
        if not os.path.exists(velodyne_drop_path):
            os.makedirs(velodyne_drop_path)
        if not os.path.exists(velodyne_jitter_path):
            os.makedirs(velodyne_jitter_path)
        else:
            print("already processed", path)
            #continue

        if depth_path_exists:
            if not os.path.exists(velodyne_depth_path):
                os.makedirs(velodyne_depth_path)

        #outpath
        if not os.path.exists(out_path):
            os.makedirs(out_path)



        rgb_path = path + 'image_2/'
        if not os.path.exists(rgb_path):
            rgb_path = path + 'image_2/'
            if not os.path.exists(rgb_path):
                rgb_path = path + 'frames/rgb/Camera_0/'

        calib_path = path + 'calib/'

        files = os.listdir(rgb_path)
        files.sort()

        files = files[:max_files]


        image_w = 2024
        image_h = 512
        fov = 120

        focal_length = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

        effective_vertical_fov = fov # * image_h/image_w 

        target_fov = 26.8

        effective_target_ratio = target_fov / effective_vertical_fov

        #get the new image height
        image_h_target = int(image_h * effective_target_ratio)

        offset = 0#24
        top_crop = (image_h - image_h_target) // 2
        bottom_crop = image_h - top_crop

        if use_multiprocessing:
            # create a pool of worker processes
            pool = multiprocessing.Pool()
            #print number of threads
            print("Number of threads", multiprocessing.cpu_count())
            # map the process_file function to each file in parallel
            #print(files)
            print(len(files), "files")
            pool.map(process_file, files)
                    # close the pool of worker processes
            pool.close()

        else:
            if file_id != -1:
                process_file(files[file_id])
            else:
                for file in files:

                    process_file(file)
