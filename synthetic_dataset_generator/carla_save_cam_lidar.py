import time
import carla
from carla import VehicleLightState as vls


import argparse
import os
from queue import Queue
import json
import logging
import numpy as np
import math
from pathlib import Path
import random
import re
import shutil
from PIL import Image, ImageDraw
from datetime import datetime
from typing import Union

from bounding_box import create_kitti_datapoint
from camera_utils import *
from mesh2blueprint import get_blueprint_from_mesh
import custom_agent



def stats_per_town(base_path):
    paths = os.listdir(base_path)
    #sort
    paths.sort()


    label_name="label_2"

    town_file_nbr_dict = {}
    town_label_nbr_dict = {}
    town_file_occurence_dict = {}

    if not "Town" in paths[0]:
        paths = [base_path]

    for path in paths:
        #folder name is of the form: 1730514080_Town01_51
        #extract town name
        label_folder = os.path.join(base_path, path, label_name)    
        if "Town" in path:
            town = path.split("_")
            #find the one that has Town in it
            for t in town:
                if "Town" in t:
                    town = t
                    break
        else:
            town = "KITTI"

        num_files = len(os.listdir(label_folder))
        num_labels = 0
        for label in os.listdir(label_folder):
            label_path = os.path.join(label_folder, label)
            with open(label_path, 'r') as f:
                num_labels += len(f.readlines())

        if town in town_file_nbr_dict:
            town_file_nbr_dict[town] += num_files
            town_label_nbr_dict[town] += num_labels
            town_file_occurence_dict[town] += 1
        else:
            town_file_nbr_dict[town] = num_files
            town_label_nbr_dict[town] = num_labels
            town_file_occurence_dict[town] = 1


        
    print("Number of files ", num_files, town)
    return town_file_nbr_dict, town_label_nbr_dict, town_file_occurence_dict

def is_truck(vehicle):
    type = vehicle.attributes['base_type']
    trucks = ['truck', 'bus']
    if type in trucks:
        return True
    else: 
        return False

# Sensor callback for all involved sensors.
# This is where you receive the sensor data and store it into a queue
def sensor_callback(sensor_data, sensor_queue):
    sensor_queue.put(sensor_data)

def spawn_props(client, nbr_props, props_list):
    world = client.get_world()
    blueprint_library = world.get_blueprint_library().filter('static.prop.*')
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    vehicle_blueprints = custom_blueprint_library(vehicle_blueprints)
    blueprint_library = [x for x in blueprint_library if not x.id.endswith('fountain')] # filter fountain because it's too big and can block traffic
    i = 0
    print(f"Number of props to spawn: {nbr_props}")
    while i < nbr_props:
        if nbr_props <= 0:
            break


        # Choose a random static prop blueprint
        #make 10 percent of random props cars

     
        # Get a random location on the sidewalk
        random_location = world.get_random_location_from_navigation()

        # Adjust the rotation of the object
        object_rotation = carla.Rotation(yaw=random.randint(0, 360))
        #probably need to add vehicles props to the bb generation function
        if random.randint(0, 100) < 0:
            static_prop_bp = random.choice(vehicle_blueprints)
                 # Spawn the object using the chosen blueprint and location
            prop = world.try_spawn_actor(static_prop_bp, carla.Transform(random_location, object_rotation))
            if prop is not None:
                prop.set_simulate_physics(False)
                # is this necessary?
        else:
            static_prop_bp = random.choice(blueprint_library)
            # Spawn the object using the chosen blueprint and location
            prop = world.try_spawn_actor(static_prop_bp, carla.Transform(random_location, object_rotation))

   
        if prop is not None:
            props_list.append(prop.id)
        i += 1

    logging.debug('Spawned %d props.' % (len(props_list)))


def custom_blueprint_library(blueprint_library, leave_big = False):
    # no motorcycles or bikes, because the respective models always have a driver attached to it
    blueprint_library = [x for x in blueprint_library if int(x.get_attribute('number_of_wheels')) == 4]
    if leave_big:
        blueprint_library = [x for x in blueprint_library if x.get_attribute('base_type') not in ['Bus', 'bus']]
    else:
        blueprint_library = [x for x in blueprint_library if x.get_attribute('base_type') not in ['Bus', 'bus', 'truck', 'van']]

    # List of abnormal vehicles
    abnormal_vehicle_list = ['isetta', 'cybertruck', 'microlino', 'fuso', 'european_hgv', 'sprinter', 't2_2021', 'fusorosa', 'wrangler_rubicon', 'charger_police_2020', 'charger_police','patrol']
    # Filter blueprints to exclude any whose id ends with any entry in abnormal_vehicle_list
    blueprint_library = [x for x in blueprint_library if not any(x.id.endswith(vehicle) for vehicle in abnormal_vehicle_list)]

    return blueprint_library

def spawn_static_parked_vehicles(client, respawn_number, actors_parked_layer):

    print(f"Number of static parked vehicles to respawn: {respawn_number} of {len(actors_parked_layer)}")
    respawn_count = 0
    world = client.get_world()    
    parked_static_list =[]
    for actor in actors_parked_layer:
        blueprints = world.get_blueprint_library().filter('vehicle.*')
        blueprints = custom_blueprint_library(blueprints)
        vehicle_bp = random.choice(blueprints)
        spawn_point_parking_car = actor.transform
        spawn_point_parking_car.location.z += 0.01
        spawn_point_parking_car.rotation.yaw += random.uniform(-8.0, 8.0)
        # Spawn the vehicle
        parking_vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_parking_car)
        if parking_vehicle is not None:
            parking_vehicle.set_simulate_physics(False)
            parking_vehicle.set_autopilot(False)
            #vehicles_list_parked.append(parking_vehicle.id)
            #side parked cars can only drive forward....
            parked_static_list.append(parking_vehicle)
            respawn_count += 1
            if respawn_count >= respawn_number:
                print("respawned enough vehicles")
                break
     

    print(f"Respawned {respawn_count} of {respawn_number} vehicles")
    return parked_static_list

def spawn_random_parking_vehicles(client, vehicle, vehicles_list, nbr_parking_vehicles, town):
    if nbr_parking_vehicles <= 0:
        return

    # get road shoulder of the right side
    world = client.get_world()
    
    #blueprint_library = sorted(blueprint_library, key=lambda bp: bp.id)
    blueprint_library = world.get_blueprint_library().filter('vehicle.*')
    blueprint_library = custom_blueprint_library(blueprint_library)

    spawn_points = world.get_map().get_spawn_points()
    print(f"Number of spawn points: {len(spawn_points)}")
    already_spawned = 0

    for point in spawn_points:
        waypoint_right_shoulder = world.get_map().get_waypoint(point.location, lane_type=carla.LaneType.Shoulder)
        if waypoint_right_shoulder is not None:
            #this breaks if only one right waypoint is found
            right_waypoints = waypoint_right_shoulder.next_until_lane_end(random.uniform(3, 6))
            parking_lane_size = random.uniform(0, 10)
            spawned_in_lane = 0
            if len(right_waypoints) <= 1 and town == 'Town15':
                break
            for waypoint in right_waypoints:
                if waypoint is not None and waypoint.lane_type == carla.LaneType.Shoulder and not waypoint.is_junction:
                    vehicle_bp = random.choice(blueprint_library)
                    spawn_point_parking_car = waypoint.transform
                    spawn_point_parking_car.location.z += 0.01
                    spawn_point_parking_car.rotation.yaw += random.uniform(-8.0, 8.0)
                    # Spawn the vehicle
                    vehicle_bp.set_attribute('role_name', 'parked')
                    parking_vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_parking_car)

                    if parking_vehicle is not None:
                        spawned_in_lane += 1
                        already_spawned += 1
                        vehicles_list.append(parking_vehicle.id)
                        parking_vehicle.set_simulate_physics(False)
                        # vehicle.set_autopilot(False)
                        if already_spawned >= nbr_parking_vehicles:
                            logging.debug('Spawned %d parking vehicles.' % already_spawned)
                            return
                        if parking_lane_size >= parking_lane_size:
                            break
    logging.debug('Spawned %d parking vehicles.' % already_spawned)
    #exit()

# extracted from generate_traffic.py of the CARLA examples
def spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_list, all_walkers_id, set_autopilot=False):
    world = client.get_world()

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)

    # traffic_manager.set_hybrid_physics_mode(True)
    # traffic_manager.set_random_device_seed(args.seed)

    traffic_manager.set_synchronous_mode(True)
    synchronous_master = True

    blueprints = world.get_blueprint_library().filter('vehicle.*')

    blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')

    safe = True
    if safe:

        blueprints = custom_blueprint_library(blueprints, leave_big=True)
        print(f"Number of blueprints: {len(blueprints)}")
      #now filter out the big vehicles
        blueprints_truck =[x for x in blueprints if x.get_attribute('base_type') in ['Bus', 'bus', 'truck']]
        blueprints_van = [x for x in blueprints if x.get_attribute('base_type') in ['van']]
        blueprints = [x for x in blueprints if x.get_attribute('base_type') not in ['Bus', 'bus', 'truck', 'van']]
       



        '''
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
        blueprints = [x for x in blueprints if not x.id.endswith('fuso')]
        blueprints = [x for x in blueprints if not x.id.endswith('fusorosa')]
        # remove vehicle blueprints where collision mesh couldn't be updated
        blueprints = [x for x in blueprints if not x.id.endswith('european_hgv')]
        blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2_2021')]
        '''



    blueprints = sorted(blueprints, key=lambda bp: bp.id)


    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if nbr_vehicles <= number_of_spawn_points:
        random.shuffle(spawn_points)
    elif nbr_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        logging.warning(msg, nbr_vehicles, number_of_spawn_points)
        nbr_vehicles = number_of_spawn_points

    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    SetVehicleLightState = carla.command.SetVehicleLightState
    FutureActor = carla.command.FutureActor

    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    vehicle_counter = 0
    for n, transform in enumerate(spawn_points):
        if n >= nbr_vehicles:
            break


        #truck random is 1 percent
        if random.randint(0, 100) < 1:
            blueprint = random.choice(blueprints_truck)

        elif random.randint(0, 100) < 5:
            blueprint = random.choice(blueprints_van)
        else:
            blueprint = random.choice(blueprints)
        #if vehicle is der vehicle.mitsubishi.fusorosa, with 90% probability, choose another vehicle
        '''while blueprint.id.endswith('fusorosa'):
         
            if random.randint(0, 100) < 90:
                blueprint = random.choice(blueprints)
                print('use '+ blueprint.id + ' instead of fuso')
            else:
                break
        '''
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        # prepare the light state of the cars to spawn
        light_state = vls.NONE
        car_lights_on = False
        if car_lights_on:
            light_state = vls.Position | vls.LowBeam | vls.LowBeam

        # spawn the cars and set their autopilot and light state all together
        batch.append(SpawnActor(blueprint, transform)
                     .then(SetAutopilot(FutureActor, set_autopilot, traffic_manager.get_port()))
                     .then(SetVehicleLightState(FutureActor, light_state)))

    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.has_error():
            logging.error(response.error)
        else:
            vehicle_counter += 1
            vehicles_list.append(response.actor_id)

    all_vehicle = world.get_actors(vehicles_list)
    for vehicle in all_vehicle:
        # update traffic manager settings for each vehicle
        percentage_lane_change_left = random.uniform(0.0, 80.0)
        percentage_lane_change_right = random.uniform(0.0, 80.0)
        percentage_speed_difference = random.uniform(-30.0, 0.0) # negative values exceed speed limit, positve values stay below speed limit
        percentage_keep_right_rule = random.uniform(0.0, 50.0)
        distance_to_leading_vehicle = random.uniform(0.2, 3.0)
        #lane_offset = random.uniform(-0.4, 0.4)
        lane_offset = random.uniform(-0.2, 0.2)

        traffic_manager.update_vehicle_lights(vehicle, True) # traffic manager updates vehicle lights
        traffic_manager.distance_to_leading_vehicle(vehicle, distance_to_leading_vehicle)
        traffic_manager.random_left_lanechange_percentage(vehicle, percentage_lane_change_left)
        traffic_manager.random_right_lanechange_percentage(vehicle, percentage_lane_change_right)
        if not is_truck(vehicle):
            traffic_manager.vehicle_lane_offset(vehicle, lane_offset)
            traffic_manager.keep_right_rule_percentage(vehicle, percentage_keep_right_rule)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, percentage_speed_difference)
        

    # -------------
    # Spawn Walkers
    # -------------

    # some settings
    walkers_list = []
    percentagePedestriansRunning = 0.0  # how many pedestrians will run
    percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road
    # 1. take all the random locations to spawn
    spawn_points = []
    all_loc = []
    i = 0
    while i < nbr_walkers:
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if ((loc != None) and not (loc in all_loc)):
            spawn_point.location = loc
            spawn_point.location.z += 0.01
            spawn_points.append(spawn_point)
            all_loc.append(loc)
            i = i + 1
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                speed_string = walker_bp.get_attribute('speed').recommended_values[1]
            else:
                speed_string = walker_bp.get_attribute('speed').recommended_values[2]

            speed_float = float(speed_string)
            speed_float/=1.5
            speed_string = str(speed_float)
            walker_speed.append(speed_string)
        else:
            logging.warning("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_walkers_id.append(walkers_list[i]["con"])
        all_walkers_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_walkers_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    # TODO: check if this is still necessary
    world.tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_walkers_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

    logging.debug('Spawned %d vehicles, %d walkers.' % (len(vehicles_list), len(walkers_list)))

def respawn_parked_vehicles_layer(client, parked_vehicles_env_objects, vehicles_list) -> list:
    
    world = client.get_world()
    debug = world.debug
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager()
    synchronous_master = True
    
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    SetVehicleLightState = carla.command.SetVehicleLightState
    FutureActor = carla.command.FutureActor

    batch = []

    vehicle_counter = 0
    for object in parked_vehicles_env_objects:
        obj_transform = carla.Transform(object.transform.location + carla.Location(0.0, 0.0, 1.0), object.transform.rotation)
        bp = get_blueprint_from_mesh(object.name)
        if bp == "other":
            #random vehicle
            obj_bp = random.choice(custom_blueprint_library(blueprint_library.filter('vehicle.*')))
        else:
            obj_bp = blueprint_library.find(bp)

            obj_bp_type = obj_bp.get_attribute('base_type')
            #print(obj_bp.id, obj_bp_type, object.name)
        
            if obj_bp_type not in ['Car', 'car']:
                print ("skipping " + obj_bp.id, obj_bp_type, obj_bp_type in ['Car', 'car'])
                #is continue enough? cars still seem to be there?

        #spawn fails anyway so we can continue to read the output
        continue

        if obj_bp.has_attribute('color'):
            color = random.choice(obj_bp.get_attribute('color').recommended_values)
            obj_bp.set_attribute('color', color)
        light_state = vls.NONE
        # spawn the cars and set their autopilot and light state all together
        actor = world.try_spawn_actor(obj_bp, obj_transform)
        # if actor is None:
        #     logging.error("Failed to spawn " + str(obj_bp) + " at " + str(obj_transform))
        #     debug.draw_point(obj_transform.location)
        # else:
        #     vehicle_counter += 1
        #     vehicles_list.append(actor.id)

        batch.append(SpawnActor(obj_bp, obj_transform)
                     .then(SetAutopilot(FutureActor, False, traffic_manager.get_port()))
                     .then(SetVehicleLightState(FutureActor, light_state)))

    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.has_error():
            logging.error(response.error)
        else:
            vehicle_counter += 1
            vehicles_list.append(response.actor_id)

    return vehicles_list



def project_lidar2camera(lidar_data, lidar, camera, K, cam_image):
    # Get the lidar data and convert it to a numpy array.
    p_cloud_size = len(lidar_data)
    p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
    p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

    # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
    # focus on the 3D points.
    intensity = np.array(p_cloud[:, 3])

    # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
    local_lidar_points = np.array(p_cloud[:, :3]).T

    # Add an extra 1.0 at the end of each 3d point so it becomes of
    # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
    local_lidar_points = np.r_[
        local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

    # This (4, 4) matrix transforms the points from lidar space to world space.
    lidar_2_world = lidar.get_transform().get_matrix()

    # Transform the points from lidar space to world space.
    world_points = np.dot(lidar_2_world, local_lidar_points)

    # This (4, 4) matrix transforms the points from world to sensor coordinates.
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # Transform the points from world space to camera space.
    sensor_points = np.dot(world_2_camera, world_points)

    # New we must change from UE4's coordinate system to a "standard"
    # camera coordinate system (the same used by OpenCV):

    # ^ z                       . z
    # |                        /
    # |              to:      +-------> x
    # | . x                   |
    # |/                      |
    # +-------> y             v y

    # This can be achieved by multiplying by the following matrix:
    # [[ 0,  1,  0 ],
    #  [ 0,  0, -1 ],
    #  [ 1,  0,  0 ]]

    # Or, in this case, is the same as swapping:
    # (x, y ,z) -> (y, -z, x)
    point_in_camera_coords = np.array([
        sensor_points[1],
        sensor_points[2] * -1,
        sensor_points[0]])

    # Finally we can use our K matrix to do the actual 3D -> 2D.
    points_2d = np.dot(K, point_in_camera_coords)

    # Remember to normalize the x, y values by the 3rd value.
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]])

    # At this point, points_2d[0, :] contains all the x and points_2d[1, :]
    # contains all the y values of our points. In order to properly
    # visualize everything on a screen, the points that are out of the screen
    # must be discarded, the same with points behind the camera projection plane.
    points_2d = points_2d.T
    # TODO: remove hardcoded values for image size
    # intensity = intensity.T
    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < 1242) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < 375) & \
        (points_2d[:, 2] > 0.0)

    points_2d = points_2d[points_in_canvas_mask]
    # intensity = intensity[points_in_canvas_mask]

    # Extract the screen coords (uv) as integers.
    u_coord = points_2d[:, 0].astype(np.int_)
    v_coord = points_2d[:, 1].astype(np.int_)

    # Extract a single scanline
    scanlines_u = []
    scanlines_v = []
    threshold = 500
    current_point_u = [u_coord[0]]
    current_point_v = [v_coord[0]]

    for i in range(1, len(u_coord)):
        if abs(u_coord[i] - u_coord[i - 1]) > threshold:
            scanlines_u.append(np.array(current_point_u))
            scanlines_v.append(np.array(current_point_v))
            current_point_u = [u_coord[i]]
            current_point_v = [v_coord[i]]
        else:
            current_point_u.append(u_coord[i])
            current_point_v.append(v_coord[i])

    # Draw the 2d points on the image as a single pixel using numpy.
    projected_image = np.copy(cam_image)

    lidar_cam_image = np.zeros([375, 1242, 3], dtype=np.uint8)

    i = 0
    for scanline in scanlines_u:
        lidar_cam_image[i, scanline] = projected_image[scanlines_v[i], scanline]
        i += 1

    return lidar_cam_image


def follow(transform, world):  # Transform carla.Location(x,y,z) from sensor to world frame
    rot = transform.rotation
    #     loc = transform.location
    rot.pitch = -25
    world.get_spectator().set_transform(carla.Transform(transform.transform(carla.Location(x=-15, y=0, z=5)), rot))


#     world.get_spectator().set_transform(carla.Transform(loc, rot))

def clean_up(world, client, original_settings, vehicle, sensor_list, vehicles_list, vehicles_list_static, all_walkers_id, props_list, do_replay=False):
    # delete all actors, sensors, vehicles, walkers
    world.apply_settings(original_settings)
    for sensor in sensor_list:
        sensor.destroy()
    if not do_replay:
        vehicle.destroy()
        print('Destroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        vehicles_list.clear()

        print('Destroying %d static vehicles' % len(vehicles_list_static))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list_static])
        vehicles_list_static.clear()

        
        # Stop walker controllers (list is [controller, actor, controller, actor ...])
        all_actors = world.get_actors(all_walkers_id)
        for i in range(0, len(all_walkers_id), 2):
            all_actors[i].stop()
        print('Destroying %d walkers' % (len(all_walkers_id) // 2))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_walkers_id])
        all_walkers_id.clear()

        print('Destroying %d props' % (len(props_list)))
        client.apply_batch([carla.command.DestroyActor(x) for x in props_list])
        props_list.clear()


def clear_directory(directory_path: Union[str, Path]) -> list:
    """Irreversibly removes all files and folders inside the specified
    directory. Returns a list with paths Python lacks permission to delete."""
    erroneous_paths = []
    try:
        for path_object in Path(directory_path).iterdir():
            try:
                if path_object.is_dir():
                    shutil.rmtree(path_object)
                else:
                    path_object.unlink()
            except PermissionError:
                erroneous_paths.append(path_object)
    except FileNotFoundError:
        logging.error("Path not found! No files were deleted.")

    return erroneous_paths


#############################
#############################

town_list = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07', 'Town10HD', 'Town15']
town_percentage = [0.11, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.12, 0.22]
#check if the sum of the percentages is 1
print(sum(town_percentage))
assert(sum(town_percentage) == 1)
small_towns = ['Town01', 'Town02', 'Town03','Town04', 'Town05', 'Town06', 'Town07']
cur_town = ''



def main():
    global cur_town
    # -------------------------------------
    # Argparse
    # -------------------------------------
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--save-data',
        action='store_true',
        help='Save KITTI like dataset during simulation.')
    argparser.add_argument(
        '--record',
        action='store_true',
        help='Record simulation run')
    argparser.add_argument(
        '--replay',
        action='store_true',
        help='Replay recorded simulation run')
    argparser.add_argument(
        '--replay-file',
        default='recording01.log',
        help='Name and path of replay file. Using \, / or : characters in the file name will define it as an absolute path')
    argparser.add_argument(
        '--record-file',
        default='recording01.log',
        help='Name and path of recorded file. Using \, / or : characters in the file name will define it as an absolute path')
    argparser.add_argument(
        '--npc-autopilot-at-startup',
        action='store_true',
        help='Save KITTI like dataset during simulation.')
    args = argparser.parse_args()
    print(args)

    # -------------------------------------
    # Settings
    # -------------------------------------

    logging.basicConfig(format='%(levelname)s: [%(asctime)s]: %(message)s', level=logging.DEBUG)
    # check replay and record args not active at the same time
    assert((args.record != args.replay) or (not args.record and not args.replay))#, f"record: {args.record}, replay: {args.replay}")
    ## Output folders

    base_dir = 'data/kitti/training/'
    base_dir = "/media/oq55olys/chonk/Datasets/kittilike/carla/kitti/training_truck/"
    #add a timestamp to the base_dir

    out_pth = "/media/oq55olys/chonk/Datasets/kittilike/carla/kitti/training_truck/"

    #get result stats
    #if path exists
    if os.path.exists(out_pth) and len(os.listdir(out_pth)) > 0:
        town_file_nbr_dict, town_label_nbr_dict, town_file_occurence_dict = stats_per_town(out_pth)

        sum_files = sum(town_file_nbr_dict.values())
        if sum_files > 0:
            percentage_diffs = []
            for town_name, target_percentage in zip(town_list, town_percentage):
                if town_name in town_file_nbr_dict:
                    real_percentage = town_file_nbr_dict[town_name] / sum_files
                else:
                    real_percentage = 0
                print(f"Town: {town_name}, Real percentage: {real_percentage}, Target percentage: {target_percentage}, Difference: {target_percentage - real_percentage}")
                percentage_diffs.append((target_percentage - real_percentage))

            #select town with biggest difference
            town = town_list[percentage_diffs.index(max(percentage_diffs))]
            print(f"Selected town: {town}")
        else:
            town = random.choices(town_list, town_percentage)[0]
            print(f"Random town: {town}")
    else:
        town = random.choices(town_list, town_percentage)[0]
        print(f"Random town: {town}")

    FixedSpawn = False
    # Create grayscale camera data via luminosity method
    luminosity_weights = [0.21, 0.72, 0.07]
    '''
    reload_map = True
    if cur_town == -1:
        town = random.choice(town_list)
        cur_town = town
    #town = 'Town15'
    else:
        town = cur_town
        reload_map = False
    '''

    if FixedSpawn:
        town = 'Town01'



    population = 250
    min_population = 30
    if town == 'Town15':
        population =  700
        min_population = 180

 


    nbr_vehicles = random.randint(min_population, population)
    nbr_parking_vehicles = random.randint(min_population*4, population*2)
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Create the directory name with the formatted time
    base_dir = base_dir + current_time + '_' + town + '_' + str(nbr_vehicles) + '/'
    path_cam00 = base_dir + 'image_0/'
    path_cam01 = base_dir + 'image_1/'
    path_cam02 = base_dir + 'image_2/'
    path_cam03 = base_dir + 'image_3/'

    path_depth = base_dir + 'depth_0/'
    path_lidar = base_dir + 'velodyne/'
    path_lidar_vel = base_dir + 'velodyne_like/'

    # Lidar as camera
    path_lidar_cam = base_dir + 'velodyne_cam/'

    path_label = base_dir + 'label_2/'
    path_calib = base_dir + 'calib/'

    # json file with actor bounding box extends
    json_path = 'actor_bounding_box_size.json'

    # Time related stuff
    fixed_delta_seconds = 0.1#  # simulation step size
    #somehow lower values increase the time between frames
    frame_id = 0  # init frame id
    frames_total = 100  # number of total datapoints to be saved

    do_record = args.record
    do_replay = args.replay
    recorder_path = args.record_file
    replay_path = args.replay_file
    parked_spawned = False

    do_save_data = args.save_data  # if True, all frames will be saved into the respective folders
    do_draw_bounding_boxes = False  # if True, the 3D bounding boxes will be drawn onto the camera 0 image

    ## Spawning objects


    #make index list of parked vehicles 0, 1, ... nbr_parking_vehicles

    nbr_props = 0

    nbr_walkers = 0# random.randint(0, 100)
    print(f"Number of walkers: {nbr_walkers}")
    if town == 'Town15':
        nbr_walkers = 0

    npc_autopilot = args.npc_autopilot_at_startup
    print(f"NPC autopilot: {npc_autopilot}")

    map = "/Game/Carla/Maps/Town15/Town15"
    # map = "/Game/Carla/Maps/Town10HD_Opt"

    # print(f"Town: {town}")
    map = "/Game/Carla/Maps/"+town
    # Sensor settings
    image_size_x = 1242  # Image width of camera images
    image_size_y = 375  # Image height of camera images
    fov = 82.59  # horizontal FOV for cameras. Vertical FOV will be scaled accordingly
    object_detection_range = 80 # meters

    # ------------------------------------

    #clear_directory(base_dir)

    os.makedirs(path_cam00)
    os.makedirs(path_cam01)
    os.makedirs(path_cam02)
    os.makedirs(path_cam03)

    os.makedirs(path_depth)
    os.makedirs(path_lidar)
    os.makedirs(path_lidar_vel)

    os.makedirs(path_lidar_cam)

    os.makedirs(path_label)
    os.makedirs(path_calib)

    vehicles_list = []
    vehicles_list_parked = []
    vehicles_list_parked_static = []
    all_walkers_id = []
    props_list = []
    vehicle = None

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)

    logging.info("----------SETTINGS----------")
    logging.info("Map: " + map)
    logging.info("Number of vehicles: " + str(nbr_vehicles))
    logging.info("Number of parking vehicles: " + str(nbr_parking_vehicles))
    logging.info("Number of props: " + str(nbr_props))
    logging.info("Number of pedestrians: " + str(nbr_walkers))
    logging.info("Sensor image width: " + str(image_size_x) + "px")
    logging.info("Sensor image height: " + str(image_size_y) + "px")
    logging.info("Sensor field of view: " + str(fov))
    logging.info("Total number of frames to be saved: " + str(frames_total))
    logging.info("Save data to disk: " + str(do_save_data))
    logging.info("Save bounding box to camera image 0: " + str(do_draw_bounding_boxes))
    logging.info("----------SETTINGS----------")

    # load data from json
    with open(json_path, 'r') as json_file:
        json_data = json_file.read()
    try:
        json_bbs = json.loads(json_data)
    except:
        logging.error("Error while parsing JSON File for bounding boxes!")
        json_bbs = None

    if json_bbs is None:
        logging.warn("JSON file is None!")

    #if reload_map:
    print(cur_town, town)
    if cur_town != town:
        print("loading new map")
        client.load_world(map) 
        cur_town = town

    
   
    world = client.get_world()
    world.set_weather(carla.WeatherParameters.CloudyNoon)


    blueprint_library = world.get_blueprint_library()
    
 
    parked_cars_static = world.get_environment_objects(carla.CityObjectLabel.Car)
    parked_trucks_static = world.get_environment_objects(carla.CityObjectLabel.Truck)
    parked_buses_static = world.get_environment_objects(carla.CityObjectLabel.Bus)
    parked_bikes_static = world.get_environment_objects(carla.CityObjectLabel.Motorcycle)
    parked_bikes_static+= world.get_environment_objects(carla.CityObjectLabel.Bicycle)
    parked_vehicles_env_obj = parked_cars_static + parked_trucks_static + parked_buses_static + parked_bikes_static
    respawn_number = len(parked_vehicles_env_obj)
    print(f"Number of static parked vehicles: {respawn_number}")
    
    #randfloat betwen 0.75 and 1
    rand_re =1# random.uniform(0.75, 1)
    if respawn_number > nbr_parking_vehicles:
        respawn_number = int(respawn_number * rand_re)

   

    actors_parked_layer = []
    parked_ids = []
    for car in parked_vehicles_env_obj:
        #destroy all parked vehicles
        parked_ids.append(car.id)
        tmp_actor = custom_agent.CustomAgent(car.transform, car.bounding_box)
        actors_parked_layer.append(tmp_actor)
    #shuffle list
    random.shuffle(actors_parked_layer)

    world.enable_environment_objects(parked_ids, False)
    # for car in parked_vehicles_env_obj:
    #     logging.warn(car)

    # Toggle all parked vehicles off & respawn layer
    print("unloading parked vehicles")
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)

    print("successfully unloaded parked vehicles")
    traffic_manager = client.get_trafficmanager(8000)


    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)  # set minimal distance to leading vehicles [m]
    traffic_manager.global_percentage_speed_difference(10.0)  # set speed difference between vehicles

    try:
        # We need to save the settings to be able to recover them at the end
        # of the script to leave the server in the same state that we found it.
        print("Setting up world")
        original_settings = world.get_settings()
        settings = world.get_settings()

        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = fixed_delta_seconds
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        if do_record:
            # ----------
            # Start recording
            client.start_recorder(recorder_path, True)
            # ----------
        if do_replay and not do_record:
            # ----------
            # Start replay
            replay_start = 0
            replay_duration = 0 # 0 = entire recording
            replay_spectator = 0 # don't focus spectator on a specific actor
            replay_spawn_sensors = False # don't spawn sensors, retrieving sensors from actor list does not work
            # Show info for relevant frames
            client.replay_file(replay_path, replay_start, replay_duration, replay_spectator, replay_spawn_sensors)    
        # ----------

        if do_replay:
            replay_info = client.show_recorder_file_info(replay_path, False)
            frame_num_replay = re.findall(r'Frames: \d+', replay_info)[0]
            frame_num_replay = int(frame_num_replay.split()[-1])
            start_frame_replay = world.get_snapshot().frame
        else:
            frame_num_replay = world.get_snapshot().frame

        # Bluepints for the sensors
        # According to KITTI sensorset
        print("Setting up sensors")
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        lidar_cam_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp = blueprint_library.find('sensor.camera.depth')
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_normals')
        lidar_vel_top_bp = blueprint_library.find('sensor.lidar.ray_cast_normals')
        lidar_vel_bot_bp = blueprint_library.find('sensor.lidar.ray_cast_normals')
        # lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

        # Used for all 4 cameras (2 rgb and 2 greyscale)
        cam_bp.set_attribute("image_size_x", str(image_size_x))
        cam_bp.set_attribute("image_size_y", str(image_size_y))
        cam_bp.set_attribute("fov", str(fov))

        depth_bp.set_attribute("image_size_x", str(image_size_x))
        depth_bp.set_attribute("image_size_y", str(image_size_y))
        depth_bp.set_attribute("fov", str(fov))

        lidar_range = object_detection_range

        lidar_bp.set_attribute('upper_fov', str(2.0))
        lidar_bp.set_attribute('lower_fov', str(-24.8))
        lidar_bp.set_attribute('channels', str(64.0))
        lidar_bp.set_attribute('range', str(lidar_range))
        lidar_bp.set_attribute('points_per_second', str(1300000))
        lidar_bp.set_attribute('rotation_frequency', str(10))
        lidar_bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
        lidar_bp.set_attribute('noise_stddev', str(0.0))
        lidar_bp.set_attribute('dropoff_general_rate', str(0.))
        lidar_bp.set_attribute('dropoff_zero_intensity', str(0.))
        lidar_bp.set_attribute('dropoff_intensity_limit', str(0.))

        # velodyne like lidar
        lidar_vel_top_bp.set_attribute('upper_fov', str(2.02984126984))
        lidar_vel_top_bp.set_attribute('lower_fov', str(2.02984126984-11.0317460317+11.0317460317/32))
        lidar_vel_top_bp.set_attribute('channels', str(32.0))
        lidar_vel_top_bp.set_attribute('range', str(lidar_range))
        lidar_vel_top_bp.set_attribute('points_per_second', str(650000))
        lidar_vel_top_bp.set_attribute('rotation_frequency', str(10))
        lidar_vel_top_bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
        lidar_vel_top_bp.set_attribute('noise_stddev', str(0.0))
        lidar_vel_top_bp.set_attribute('dropoff_general_rate', str(0.))
        lidar_vel_top_bp.set_attribute('dropoff_zero_intensity', str(0.))
        lidar_vel_top_bp.set_attribute('dropoff_intensity_limit', str(0.))

        lidar_vel_bot_bp.set_attribute('upper_fov', str(-8.799812))
        lidar_vel_bot_bp.set_attribute('lower_fov', str(-8.799812-16.541+ 16.541/32))
        lidar_vel_bot_bp.set_attribute('channels', str(32.0))
        lidar_vel_bot_bp.set_attribute('range', str(120.0))
        lidar_vel_bot_bp.set_attribute('points_per_second', str(650000))
        lidar_vel_bot_bp.set_attribute('rotation_frequency', str(10))
        lidar_vel_bot_bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
        lidar_vel_bot_bp.set_attribute('noise_stddev', str(0.0))
        lidar_vel_bot_bp.set_attribute('dropoff_general_rate', str(0.))
        lidar_vel_bot_bp.set_attribute('dropoff_zero_intensity', str(0.))
        lidar_vel_bot_bp.set_attribute('dropoff_intensity_limit', str(0.))

        # additional (depth) cam mimicking the Velodyne HDL-64E LiDAR
        lidar_cam_bp.set_attribute("image_size_x", str(2024))
        lidar_cam_bp.set_attribute("image_size_y", str(512))
        lidar_cam_bp.set_attribute("fov", str(120))


        if do_replay: # get ego-vehicle from replay
            while vehicle is None:
                all_actors = world.get_actors().filter('*vehicle*')
                for actor in all_actors:
                    if any(actor.attributes['role_name'] == x for x in ['ego_vehicle', 'hero']):
                        logging.info(f"Ego-vehicle: {actor}")
                        vehicle = actor
                world.tick()
        else: # spawn ego-vehicle and set to autopilot
            print("Spawning ego vehicle")
            spawn_points = world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points)
            if FixedSpawn:
                spawn_point = spawn_points[0]
            vhcl_bp = blueprint_library.filter("vehicle.lincoln.mkz_2017")[0]

            vhcl_bp.set_attribute('role_name', 'ego_vehicle')
            vehicle = world.spawn_actor(blueprint=vhcl_bp,
                                        transform=spawn_point)  # world.get_map().get_spawn_points()[0])

            vehicle.set_autopilot(True)
            vehicle.set_simulate_physics(True)
            
       


        # We create all the sensors and keep them in a list for convenience.
        sensor_list = []

        # The sensor data will be saved in thread-safe Queues
        cam00_queue = Queue()
        cam01_queue = Queue()
        cam02_queue = Queue()
        cam03_queue = Queue()

        # image_queue = Queue()
        depth00_queue = Queue()
        lidar_queue = Queue()
        lidar_vel_top_queue = Queue()
        lidar_vel_bot_queue = Queue()
        lidar_cam_queue = Queue()

        # Cameras
        # see https://www.cvlibs.net/datasets/kitti/setup.php as reference

        ground_offset = 0.0
        sensor_height = 1.65 + ground_offset
        lidar_offset= 0.08 #from kitti spec
  
   
        
        top_offset = 0.202
        bot_offset = 0.121
        lidar_center_offset = (top_offset+bot_offset)/2
        top_offset -= lidar_center_offset
        bot_offset -= lidar_center_offset
        #lidar_offset+=lidar_center_offset
        #there might be an offset between sensor position and optical center in kitti but that could already be correced in the velodyne sensor, so we do not use this offset

        print(f"Top offset: {top_offset}")
        print(f"Bot offset: {bot_offset}")
        print(f"Lidar offset: {lidar_offset}")
        
        USE_SENSORS = True

        if USE_SENSORS:
            # greyscale cameras
            # camera 0 is the reference camera

            sensor_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)


            pose_list = []
            cam00_pose = carla.Transform(carla.Location(x=1.03, y=0.0, z=sensor_height), sensor_rotation)
            cam_bp.set_attribute('role_name', 'cam00')
            cam00 = world.spawn_actor(cam_bp, cam00_pose, attach_to=vehicle)
            cam00.listen(lambda data: sensor_callback(data, cam00_queue))
            sensor_list.append(cam00)

            cam01_pose = carla.Transform(carla.Location(x=1.03, y=0.54, z=sensor_height), sensor_rotation)
            cam_bp.set_attribute('role_name', 'cam01')
            cam01 = world.spawn_actor(cam_bp, cam01_pose, attach_to=vehicle)
            cam01.listen(lambda data: sensor_callback(data, cam01_queue))
            sensor_list.append(cam01)

            # rgb cameras

            cam02_pose = carla.Transform(carla.Location(x=1.03, y=-0.06, z=sensor_height), sensor_rotation)
            cam_bp.set_attribute('role_name', 'cam02')
            cam02 = world.spawn_actor(cam_bp, cam02_pose, attach_to=vehicle)
            cam02.listen(lambda data: sensor_callback(data, cam02_queue))
            sensor_list.append(cam02)

            cam03_pose = carla.Transform(carla.Location(x=1.03, y=0.48, z=sensor_height), sensor_rotation)
            cam_bp.set_attribute('role_name', 'cam03')
            cam03 = world.spawn_actor(cam_bp, cam03_pose, attach_to=vehicle)
            cam03.listen(lambda data: sensor_callback(data, cam03_queue))
            sensor_list.append(cam03)
            
            #pose in kitti does not take account that optical center is not in the center of the lidar, this used for calculating velo2cam
            lidar01_pose_ref = carla.Transform(carla.Location(x=0.76, y=0.0, z=sensor_height+lidar_offset), sensor_rotation)

            lidar01_pose = carla.Transform(carla.Location(x=0.76, y=0.0, z=sensor_height+lidar_offset), sensor_rotation)
            lidar_bp.set_attribute('role_name', 'lidar01')
            lidar01 = world.spawn_actor(lidar_bp, lidar01_pose, attach_to=vehicle)
            lidar01.listen(lambda data: sensor_callback(data, lidar_queue))
            sensor_list.append(lidar01)

            lidar_vel_top_pose = carla.Transform(carla.Location(x=0.76, y=0.0, z=sensor_height+lidar_offset+top_offset), sensor_rotation)
            lidar_vel_top_bp.set_attribute('role_name', 'lidar_vel_top')
            lidar_vel_top = world.spawn_actor(lidar_vel_top_bp, lidar_vel_top_pose, attach_to=vehicle)
            lidar_vel_top.listen(lambda data: sensor_callback(data, lidar_vel_top_queue))
            sensor_list.append(lidar_vel_top)

            lidar_vel_bot_pose = carla.Transform(carla.Location(x=0.76, y=0.0, z=sensor_height+lidar_offset+bot_offset), sensor_rotation)
            lidar_vel_bot_bp.set_attribute('role_name', 'lidar_vel_bot')
            lidar_vel_bot = world.spawn_actor(lidar_vel_bot_bp, lidar_vel_bot_pose, attach_to=vehicle)
            lidar_vel_bot.listen(lambda data: sensor_callback(data, lidar_vel_bot_queue))
            sensor_list.append(lidar_vel_bot)

            # depth cam based on cam0
            depth_bp.set_attribute('role_name', 'cam_depth00')
            cam00_depth = world.spawn_actor(depth_bp, cam02_pose, attach_to=vehicle)
            cam00_depth.listen(lambda data: sensor_callback(data, depth00_queue))
            sensor_list.append(cam00_depth)

            lidar_cam_bp.set_attribute('role_name', 'lidar01_cam')
            lidar01_cam = world.spawn_actor(lidar_cam_bp, lidar01_pose, attach_to=vehicle)
            lidar01_cam.listen(lambda data: sensor_callback(data, lidar_cam_queue))
            sensor_list.append(lidar01_cam)
            
            pose_list.append(cam00_pose)
            pose_list.append(cam01_pose)
            pose_list.append(cam02_pose)
            pose_list.append(cam03_pose)
            pose_list.append(lidar01_pose)
            pose_list.append(lidar_vel_top_pose)
            pose_list.append(lidar_vel_bot_pose)
            pose_list.append(cam02_pose)
            pose_list.append(lidar01_pose)
    

      

        if not do_replay:
            

            if True:
                print("Spawning static parked vehicles...")
                vehicles_list_parked_static= spawn_static_parked_vehicles(client, respawn_number, actors_parked_layer)
        
            print(nbr_parking_vehicles, "active parted vehicles targeted")
 
            spawn_random_parking_vehicles(client, vehicle, vehicles_list_parked, nbr_parking_vehicles, town)
            
            if town != "Town03" and town != "Town15":
                spawn_props(client, nbr_props, props_list)
   
            spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_list, all_walkers_id, npc_autopilot)

            nbr_spawned_parked  = len(vehicles_list_parked)
            #parked indices
            parked_driving_list = list(range(nbr_spawned_parked))
            npc_vehicles = world.get_actors(vehicles_list)
            parked_vehicles = world.get_actors(vehicles_list_parked)
            vehicles_list += vehicles_list_parked
            all_vehicles = world.get_actors(vehicles_list)
                   
       
        else:
            world.tick() # spawn_npc ticks world when spawning
        

        # -------------------------------------
        # Camera parameters
        # -------------------------------------

        K = get_intrinsic_camera_parameters(cam_bp)
        K_b = get_intrinsic_camera_parameters(cam_bp, True)
        if USE_SENSORS:  



            P0_KITTI = get_projection_matrix(K, cam00_pose.location.y - cam00_pose.location.y)
            P1_KITTI = get_projection_matrix(K, cam00_pose.location.y - cam01_pose.location.y)
            P2_KITTI = get_projection_matrix(K, cam00_pose.location.y - cam02_pose.location.y)
            P3_KITTI = get_projection_matrix(K, cam00_pose.location.y - cam03_pose.location.y)
            P0_KITTI_b = get_projection_matrix(K_b, cam00_pose.location.y - cam00_pose.location.y)
            P1_KITTI_b = get_projection_matrix(K_b, cam00_pose.location.y - cam01_pose.location.y)
            P2_KITTI_b = get_projection_matrix(K_b, cam00_pose.location.y - cam02_pose.location.y)
            P3_KITTI_b = get_projection_matrix(K_b, cam00_pose.location.y - cam03_pose.location.y)

            # Calculate transformation matrix from imu to lidar

            translation_vector_cam00 = np.array(
                [cam00_pose.location.x, cam00_pose.location.y, cam00_pose.location.z])

            translation_vector_cam01 = np.array(
                [cam01_pose.location.x, cam01_pose.location.y, cam01_pose.location.z])

            translation_vector_cam02 = np.array(
                [cam02_pose.location.x, cam02_pose.location.y, cam02_pose.location.z])

            translation_vector_cam03 = np.array(
                [cam03_pose.location.x, cam03_pose.location.y, cam03_pose.location.z])

            average_translation_vector = np.mean([translation_vector_cam00,
                                                translation_vector_cam01,
                                                translation_vector_cam02,
                                                translation_vector_cam03], axis=0)

            #Tr_imu_to_velo_KITTI = np.eye(4)

            #Tr_imu_to_velo_KITTI[:3, 3] = average_translation_vector



            # Rectification matrix
            R0_rect = np.identity(3)
        
            Tr_velo_to_cam = calculate_transformation_matrix(lidar01_pose_ref, cam00_pose)

            # Tr_imu_to_velo_KITTI = Tr_imu_to_velo
            Tr_velo_to_cam_KITTI = convert_2_y_up_coord_system(Tr_velo_to_cam)
        
        ticks = 0
        time_elapsed = 0
        ticks_total = round(frames_total / fixed_delta_seconds) 

        early_starters = 10

        # -------------------------------------
        # Main loop
        # -------------------------------------
        location_now = None
        location_last = None
        if do_replay:
            frame_num_diff = world.get_snapshot().frame - start_frame_replay
        else:
            frame_num_diff = 0
        started_npc_autopilot = False
        while (frame_id < frames_total) and (frame_num_diff < frame_num_replay) and ticks < ticks_total:
            if do_replay:
                frame_num_diff = world.get_snapshot().frame - start_frame_replay
            else:
                frame_num_diff = 0
                if  not npc_autopilot and time_elapsed>0.4 and started_npc_autopilot ==False:#ticks == 3:
                    print("Setting autopilot of NPCs...")
                    started_npc_autopilot = True
                    for car in npc_vehicles:
                        car.set_autopilot(True)
                        car.set_autopilot = True
                if time_elapsed >0.4:
                    #random number between 30 and nbr_parking_vehicles
                    random_start = random.randint(0, nbr_parking_vehicles)*100
                    #print("Random start: " + str(random_start), "Parked vehicles: " + str(len(parked_driving_list)))
                    if (random_start < len(parked_driving_list) or early_starters>0)and len(parked_driving_list) > 0:
                        early_starters -= 1
                        parking_idx = random.choice(parked_driving_list)
                        print("Starting parked vehicle idx: " + str(parking_idx))
                        parked_driving_list.remove(parking_idx)
                        parked_vehicle = parked_vehicles[parking_idx]
                        parked_vehicle.set_autopilot(True)
                        parked_vehicle.set_autopilot = True

                #ticks += 1
            #logging.debug("Current tick: " + str(ticks) + " of " + str(ticks_total))
            time_elapsed += settings.fixed_delta_seconds
            world.tick()
            #follow(vehicle.get_transform(), world)
            w_frame = world.get_snapshot().frame
            if do_replay: # get_velocity return 0.0 in replay even though it shouldn't, get speed from location difference
                location_last = location_now
                location_now = vehicle.get_location()
                if location_last is not None:
                    speed = location_now.distance(location_last) / settings.fixed_delta_seconds
                else:
                    speed = 0.0
            else:
                velocity_veh = vehicle.get_velocity()
                speed = math.sqrt(velocity_veh.x ** 2 + velocity_veh.y ** 2)

            ''' TODO: check this
            if not parked_spawned:
            all_vehicles = world.get_actors().filter('*vehicle*')
            for actor in all_vehicles:
                if actor.attributes['role_name'] == 'parked':
                    parked_spawned = True
                    break
            '''
 

            # Get all the sensor data
            if USE_SENSORS:
                    # greyscale cameras
                 # camera 0 is the reference camera

                #up should be between -2 and 2 with gaussian distribution, so do not assign angle but gaussian distribution
                rpitch = np.random.normal(0, 0.5)
                #right should be between -15 and 15 with gaussian distribution
                ryaw1 = np.random.normal(0, 15)
                ryaw2 = np.random.normal(0, 2)
                ryaw = np.random.choice([ryaw1, ryaw2], p=[0.3, 0.7])
                #skew should be between -2 and 2 with gaussian distribution
                rroll = np.random.normal(0, 1.5)

                #print lidar01 rotation before
                #print(lidar01.get_transform().rotation, "before")
                sensor_rotation = carla.Rotation(pitch=rpitch, yaw=ryaw, roll=rroll)
                for sensor, pose in zip(sensor_list, pose_list):
                    sensor.set_transform(carla.Transform(pose.location, sensor_rotation))


                #world.tick()
                cam00_data = cam00_queue.get(True, 1.0)
                cam01_data = cam01_queue.get(True, 1.0)
                cam02_data = cam02_queue.get(True, 1.0)
                cam03_data = cam03_queue.get(True, 1.0)
                depth00_data = depth00_queue.get(True, 1.0)
                lidar_data = lidar_queue.get(True, 1.0)
                lidar_vel_top_data = lidar_vel_top_queue.get(True, 1.0)
                lidar_vel_bot_data = lidar_vel_bot_queue.get(True, 1.0)
                lidar_cam_data = lidar_cam_queue.get(True, 1.0)

                #print(lidar01.get_transform().rotation, "after")
                #get world transform of the velodyne sensor
                lidar01_pose_world = lidar01.get_transform()
                #print translation vector
                #print(lidar01_pose_world.location)
                Tr_imu_to_velo_KITTI = lidar01_pose_world.get_matrix()
                #invert matrix
                Tr_imu_to_velo_KITTI = np.linalg.inv(Tr_imu_to_velo_KITTI)
                #conver to y-up

                # Wait for all sensors to process the data
                while not (cam00_data.frame ==
                        cam01_data.frame ==
                        cam02_data.frame ==
                        cam03_data.frame ==
                        depth00_data.frame ==
                        lidar_data.frame ==
                        lidar_vel_top_data.frame ==
                        lidar_vel_bot_data.frame ==
                        lidar_cam_data.frame ==
                        w_frame):
                    print("Waiting for sensor data...")
                    cam00_data = cam00_queue.get(True, 1.0)
                    cam01_data = cam01_queue.get(True, 1.0)
                    cam02_data = cam02_queue.get(True, 1.0)
                    cam03_data = cam03_queue.get(True, 1.0)
                    depth00_data = depth00_queue.get(True, 1.0)
                    lidar_data = lidar_queue.get(True, 1.0)
                    lidar_vel_top_data = lidar_vel_top_queue.get(True, 1.0)
                    lidar_vel_bot_data = lidar_vel_bot_queue.get(True, 1.0)
                    lidar_cam_data = lidar_cam_queue.get(True, 1.0)


            datapoints = []
            vehicle_bb_3D = []
            vehicle_bb_2D = []
            array_cam02 = None
            if do_save_data and speed > 0.1 and time_elapsed >= 1.0:
                print("Processing data...")
                ticks = 0
                # Depth camera

                # Work with raw depth (encoded in RGB channels) data,
                # see https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera

                array_depth = np.frombuffer(depth00_data.raw_data, dtype=np.dtype("uint8"))
                array_depth = np.reshape(array_depth, (depth00_data.height, depth00_data.width, 4))
                # deletes the alpha channel and inverts the array (BGR => RGB)
                array_depth = array_depth[:, :, :3][:, :, ::-1]

                # Convert to 1 channel grayscale depth
                depth = array_depth.astype(np.float32)
                normalized_depth = np.dot(depth[:, :, :3], [1.0, 256.0, 65536.0])
                normalized_depth /= 16777215.0
                depth = normalized_depth * 1000
                #depth = np.uint8(depth)
                #depth = 255 - depth
                          # For debugging: Draw 3D BB on cam 0 image

                # process rgb camera data cam
                array_cam03 = np.frombuffer(cam03_data.raw_data, dtype=np.dtype("uint8"))
                array_cam03 = np.reshape(array_cam03, (cam03_data.height, cam03_data.width, 4))
                array_cam03 = array_cam03[:, :, :3][:, :, ::-1]

                array_cam02 = np.frombuffer(cam02_data.raw_data, dtype=np.dtype("uint8"))
                array_cam02 = np.reshape(array_cam02, (cam02_data.height, cam02_data.width, 4))
                array_cam02 = array_cam02[:, :, :3][:, :, ::-1]
                # Only turned to greyscale for saving memory, because Cam02 is actually a rgb camera
                # array_cam02 = np.dot(array_cam02[..., :3], luminosity_weights).astype(np.uint8)

                # Greyscale cameras
                array_cam00 = np.frombuffer(cam00_data.raw_data, dtype=np.dtype("uint8"))
                array_cam00 = np.reshape(array_cam00, (cam00_data.height, cam00_data.width, 4))
                array_cam00 = array_cam00[:, :, :3][:, :, ::-1]
                array_cam00 = np.dot(array_cam00[..., :3], luminosity_weights).astype(np.uint8)
                array_cam01 = np.frombuffer(cam01_data.raw_data, dtype=np.dtype("uint8"))
                array_cam01 = np.reshape(array_cam01, (cam01_data.height, cam01_data.width, 4))
                array_cam01 = array_cam01[:, :, :3][:, :, ::-1]
                array_cam01 = np.dot(array_cam01[..., :3], luminosity_weights).astype(np.uint8)

                # process lidar cam
                array_lidar_cam = np.frombuffer(lidar_cam_data.raw_data, dtype=np.dtype("uint8"))
                array_lidar_cam = np.reshape(array_lidar_cam, (lidar_cam_data.height, lidar_cam_data.width, 4))
                array_lidar_cam = array_lidar_cam[:, :, :3][:, :, ::-1]

                # process bounding boxes
                vehicle_transform = vehicle.get_transform()
                vehicle_location = vehicle_transform.location
                vehicle_forward_vec = vehicle_transform.get_forward_vector()
                print("Processing active bounding boxes...", len(all_vehicles))
                v = 0
                for npc in all_vehicles:
                    #print(v)
                    v += 1
                    #test performance with alive?
                    if npc.id != vehicle.id: #and npc.is_alive 
                        # Calculate the dot product between the forward vector
                        # of the vehicle and the vector between the vehicle
                        # and the other vehicle. We threshold this dot product
                        # to limit to calculation to vehicles IN FRONT OF THE CAMERA
                        #TODO: check if this is correct
                        #npc_transform = npc.transform if isinstance(npc, carla.EnvironmentObject) else npc.get_transform()
                        npc_transform = npc.get_transform()

                        # filter out if BB is in camera detection range
                        if npc_transform.location.distance(vehicle_location) <= object_detection_range:
                            ray = npc_transform.location - vehicle_location
                            if vehicle_forward_vec.dot(ray) > 0:
                                #TODO: check if cam00 or cam02 is correct
                                image, datapoint, camera_bbox, bbox_2d = create_kitti_datapoint(npc, cam00, P0_KITTI[:, :3], P0_KITTI_b[:, :3],
                                                                                    array_cam00,
                                                                                    depth, vehicle_transform, json_bbs, object_detection_range+10.)
                                #print("Datapoint: ", datapoint)
                                if datapoint is not None:
                                    vehicle_bb_3D.append(camera_bbox)
                                    vehicle_bb_2D.append(bbox_2d)
                                    datapoints.append(datapoint)
                                    #print("Skipping vehicles after" + str(v) + " vehicles")
                                    #break
                print("Processing pedestrians...")
                for ped in world.get_actors().filter('*pedestrian*'):

                    ray = ped.get_location() - vehicle_location
                    if vehicle_forward_vec.dot(ray) > 0:
                        image, datapoint, camera_bbox, bbox_2d = create_kitti_datapoint(ped, cam00, P0_KITTI[:, :3], P0_KITTI_b[:, :3], array_cam00,
                                                                            depth, vehicle_transform, json_bbs, object_detection_range+10.)
                        if datapoint is not None:
                            if ped.get_location().distance(vehicle.get_location()) <= object_detection_range:
                                datapoints.append(datapoint)
                '''
                print("Processing parked layer...")
         
                print("# Parked vehicles layer: " + str(len(actors_parked_layer)))
                for car in actors_parked_layer:
                    print(car.bounding_box)

                    if car.get_location().distance(vehicle.get_location()) <= object_detection_range:
                        image, datapoint, camera_bbox, bbox_2d = create_kitti_datapoint(car, cam02, P0_KITTI[:, :3], P0_KITTI_b[:, :3], array_cam02,
                                                                            depth, vehicle.get_transform(), json_bbs, object_detection_range+10.)
                        if datapoint is not None:
                            print("Parked car data point...")
                            print(datapoint)
                            datapoints.append(datapoint)
                '''
                print("Processing static parked vehicles...", len(vehicles_list_parked_static))
                for parked in vehicles_list_parked_static:
                    #print(parked)

                    ray = parked.get_location() - vehicle_location
                    if vehicle_forward_vec.dot(ray)>0 and parked.get_location().distance(vehicle.get_location())<= object_detection_range:
                        image, datapoint, camera_bbox, bbox_2d = create_kitti_datapoint(parked, cam00, P0_KITTI[:, :3], P0_KITTI_b[:, :3],
                                                                                    array_cam00,
                                                                                    depth, vehicle_transform, json_bbs, object_detection_range+10.)
                        if datapoint is not None:
                            vehicle_bb_3D.append(camera_bbox)
                            vehicle_bb_2D.append(bbox_2d)
                            datapoints.append(datapoint)


                # process lidar pointcloud
                print("Processing lidar data...")
                # lidar_data.save_to_disk('lidar_output/%.6d.ply' % frame_id)
                points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 14), 14))
                # points = np.reshape(points, (int(points.shape[0] / 7), 7))
                # points = np.reshape(points, (int(points.shape[0] / 4), 4))
                points = np.copy(points)
                points[:, 1] = -points[:, 1]
                points[:, 8] = -points[:, 8]
                lidar_array = np.array(points).astype(np.float32)

                # process velodyne like lidar pointcloud
                # lidar_data.save_to_disk('lidar_output/%.6d.ply' % frame_id)
                points_vel_top = np.frombuffer(lidar_vel_top_data.raw_data, dtype=np.dtype('f4'))
                points_vel_top = np.reshape(points_vel_top, (int(points_vel_top.shape[0] / 14), 14))
                # points = np.reshape(points, (int(points.shape[0] / 7), 7))
                # points = np.reshape(points, (int(points.shape[0] / 4), 4))
                points_vel_top = np.copy(points_vel_top)
                points_vel_top[:, 1] = -points_vel_top[:, 1]
                points_vel_top[:, 8] = -points_vel_top[:, 8]
                lidar_vel_array = np.array(points_vel_top).astype(np.float32)
                
                points_vel_bot = np.frombuffer(lidar_vel_bot_data.raw_data, dtype=np.dtype('f4'))
                points_vel_bot = np.reshape(points_vel_bot, (int(points_vel_bot.shape[0] / 14), 14))
                # points = np.reshape(points, (int(points.shape[0] / 7), 7))
                # points = np.reshape(points, (int(points.shape[0] / 4), 4))
                points_vel_bot = np.copy(points_vel_bot)
                points_vel_bot[:, 1] = -points_vel_bot[:, 1]
                points_vel_bot[:, 8] = -points_vel_bot[:, 8]
                lidar_vel_array_bot = np.array(points_vel_bot).astype(np.float32)
                lidar_vel_array = np.concatenate((lidar_vel_array, lidar_vel_array_bot),axis=0)

                print("Processing data done, # datapoints: ", len(datapoints))

       

            if ticks % 10 == 0:
                    print("speed: ", int(speed), "time_elapsed: ", time_elapsed, "ticks without action: ", ticks, "ticks_total: ", ticks_total)
                    logging.debug("no new boxes found")
            ticks += 1
            if do_save_data and (len(datapoints) > 0):
                print("Saving data...")
                time_elapsed = 0  # reset time since last recording

                # Save image and lidar to disk

                lidar_cam_img = Image.fromarray(array_lidar_cam)
                
                if do_draw_bounding_boxes:
    
                    cam00_img = Image.fromarray(array_cam00)
                    draw = ImageDraw.Draw(cam00_img)
                    print("Drawing bounding boxes...", len(vehicle_bb_3D), len(vehicle_bb_2D))
                    for bbox in vehicle_bb_3D:
                        points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                        # base
                        draw.line((points[0][0], points[0][1], points[1][0], points[1][1]), fill=255, width=2)
                        draw.line((points[1][0], points[1][1], points[2][0], points[2][1]), fill=255, width=2)
                        draw.line((points[2][0], points[2][1], points[3][0], points[3][1]), fill=255, width=2)
                        draw.line((points[3][0], points[3][1], points[0][0], points[0][1]), fill=255, width=2)

                        # top
                        draw.line((points[4][0], points[4][1], points[5][0], points[5][1]), fill=255, width=2)
                        draw.line((points[5][0], points[5][1], points[6][0], points[6][1]), fill=255, width=2)
                        draw.line((points[6][0], points[6][1], points[7][0], points[7][1]), fill=255, width=2)
                        draw.line((points[7][0], points[7][1], points[4][0], points[4][1]), fill=255, width=2)

                        # base to top connections
                        draw.line((points[0][0], points[0][1], points[4][0], points[4][1]), fill=255, width=2)
                        draw.line((points[1][0], points[1][1], points[5][0], points[5][1]), fill=255, width=2)
                        draw.line((points[2][0], points[2][1], points[6][0], points[6][1]), fill=255, width=2)
                        draw.line((points[3][0], points[3][1], points[7][0], points[7][1]), fill=255, width=2)

                    for bbox in vehicle_bb_2D:
                        x_min, y_min, x_max, y_max = bbox
                        draw.line((int(x_min),int(y_min), int(x_max),int(y_min)), fill=128, width=3)
                        draw.line((int(x_min),int(y_max), int(x_max),int(y_max)), fill=128, width=3)
                        draw.line((int(x_min),int(y_min), int(x_min),int(y_max)), fill=128, width=3)
                        draw.line((int(x_max),int(y_min), int(x_max),int(y_max)), fill=128, width=3)


                    cam00_img.save(path_cam00 + '%.6d.png' % frame_id)
                # Uncomment respective line, if saving of a certain sensor image is not needed
                #Image.fromarray(array_cam01).save(path_cam01 + '%.6d.png' % frame_id)

                #Image.fromarray(array_cam02).resize((100,30),0).save(path_cam02 + '%.6d.png' % frame_id)
                #print("Saved cam02 to path: " + path_cam02 + '%.6d.png' % frame_id)
                Image.fromarray(array_cam02).save(path_cam02 + '%.6d.png' % frame_id)
                #Image.fromarray(array_cam03).save(path_cam03 + '%.6d.png' % frame_id)
                #Image.fromarray(array_depth).save(path_depth + '%.6d.png' % frame_id)

                lidar_cam_img.save(path_lidar_cam + '%.6d.png' % frame_id)
     

                lidar_array.tofile(path_lidar + '%06d.bin' % frame_id)
                lidar_vel_array.tofile(path_lidar_vel + '%06d.bin' % frame_id)

                # Save calibration data to file in KITTI format
                # (only one camera on vehicle but saved 4 times to fit format)
                with open(path_calib + '%06d.txt' % frame_id, 'w') as calib_file:
                    # write camera projection matrices
                    calib_file.write('P0: ')
                    calib_file.write(
                        str(P0_KITTI[0][0]) + " " + str(P0_KITTI[0][1]) + " " + str(P0_KITTI[0][2]) + " " + str(
                            P0_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(P0_KITTI[1][0]) + " " + str(P0_KITTI[1][1]) + " " + str(P0_KITTI[1][2]) + " " + str(
                            P0_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(P0_KITTI[2][0]) + " " + str(P0_KITTI[2][1]) + " " + str(P0_KITTI[2][2]) + " " + str(
                            P0_KITTI[2][3]) + "\n")
                    calib_file.write('P1: ')
                    calib_file.write(
                        str(P1_KITTI[0][0]) + " " + str(P1_KITTI[0][1]) + " " + str(P1_KITTI[0][2]) + " " + str(
                            P1_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(P1_KITTI[1][0]) + " " + str(P1_KITTI[1][1]) + " " + str(P1_KITTI[1][2]) + " " + str(
                            P1_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(P1_KITTI[2][0]) + " " + str(P1_KITTI[2][1]) + " " + str(P1_KITTI[2][2]) + " " + str(
                            P1_KITTI[2][3]) + "\n")
                    calib_file.write('P2: ')
                    calib_file.write(
                        str(P2_KITTI[0][0]) + " " + str(P2_KITTI[0][1]) + " " + str(P2_KITTI[0][2]) + " " + str(
                            P2_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(P2_KITTI[1][0]) + " " + str(P2_KITTI[1][1]) + " " + str(P2_KITTI[1][2]) + " " + str(
                            P2_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(P2_KITTI[2][0]) + " " + str(P2_KITTI[2][1]) + " " + str(P2_KITTI[2][2]) + " " + str(
                            P2_KITTI[2][3]) + "\n")
                    calib_file.write('P3: ')
                    calib_file.write(
                        str(P3_KITTI[0][0]) + " " + str(P3_KITTI[0][1]) + " " + str(P3_KITTI[0][2]) + " " + str(
                            P3_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(P3_KITTI[1][0]) + " " + str(P3_KITTI[1][1]) + " " + str(P3_KITTI[1][2]) + " " + str(
                            P3_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(P3_KITTI[2][0]) + " " + str(P3_KITTI[2][1]) + " " + str(P3_KITTI[2][2]) + " " + str(
                            P3_KITTI[2][3]) + "\n")
                    calib_file.write('R0_rect: ')
                    calib_file.write(
                        str(R0_rect[0][0]) + " " + str(R0_rect[0][1]) + " " + str(R0_rect[0][2]) + " ")
                    calib_file.write(
                        str(R0_rect[1][0]) + " " + str(R0_rect[1][1]) + " " + str(R0_rect[1][2]) + " ")
                    calib_file.write(
                        str(R0_rect[2][0]) + " " + str(R0_rect[2][1]) + " " + str(R0_rect[2][2]) + "\n")
                    calib_file.write('Tr_velo_to_cam: ')
                    calib_file.write(
                        str(Tr_velo_to_cam_KITTI[0][0]) + " " + str(Tr_velo_to_cam_KITTI[0][1]) + " " + str(
                            Tr_velo_to_cam_KITTI[0][2]) + " " + str(Tr_velo_to_cam_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(Tr_velo_to_cam_KITTI[1][0]) + " " + str(Tr_velo_to_cam_KITTI[1][1]) + " " + str(
                            Tr_velo_to_cam_KITTI[1][2]) + " " + str(Tr_velo_to_cam_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(Tr_velo_to_cam_KITTI[2][0]) + " " + str(Tr_velo_to_cam_KITTI[2][1]) + " " + str(
                            Tr_velo_to_cam_KITTI[2][2]) + " " + str(Tr_velo_to_cam_KITTI[2][3]) + "\n")
                    calib_file.write('Tr_imu_to_velo: ')
                    calib_file.write(
                        str(Tr_imu_to_velo_KITTI[0][0]) + " " + str(Tr_imu_to_velo_KITTI[0][1]) + " " + str(
                            Tr_imu_to_velo_KITTI[0][2]) + " " + str(Tr_imu_to_velo_KITTI[0][3]) + " ")
                    calib_file.write(
                        str(Tr_imu_to_velo_KITTI[1][0]) + " " + str(Tr_imu_to_velo_KITTI[1][1]) + " " + str(
                            Tr_imu_to_velo_KITTI[1][2]) + " " + str(Tr_imu_to_velo_KITTI[1][3]) + " ")
                    calib_file.write(
                        str(Tr_imu_to_velo_KITTI[2][0]) + " " + str(Tr_imu_to_velo_KITTI[2][1]) + " " + str(
                            Tr_imu_to_velo_KITTI[2][2]) + " " + str(Tr_imu_to_velo_KITTI[2][3]) + "\n")

                with open(path_label + '%06d.txt' % frame_id, 'w') as labels_file:
                    for datapoint in datapoints:
                        labels_file.write(str(datapoint) + "\n")

                frame_id += 1  # increment frame id to have sequential frame numbers
                if False:
                    logging.debug("Saved sensor data to disk.")


                    # At this point, we have the synchronized information from the sensors.
                    logging.debug("Simulation frame: " + str(frame_id))
                    logging.debug("Camera 0 frame: " + str(cam00_data.frame))
                    logging.debug("Camera 1 frame: " + str(cam01_data.frame))
                    logging.debug("Camera 2 frame: " + str(cam02_data.frame))
                    logging.debug("Camera 3 frame: " + str(cam03_data.frame))
                    logging.debug("Lidar frame: " + str(lidar_data.frame))
                else:

                    logging.debug("Written sensor data to disk, number of BBs: "+ str(len(datapoints)))


            

    finally:
        if do_record:
            # ----------
            # Start recording
            client.stop_recorder()
            # ----------
        clean_up(world, client, original_settings, vehicle, sensor_list, vehicles_list,vehicles_list_parked_static ,all_walkers_id, props_list, do_replay)


if __name__ == "__main__":
    try:
        for i in range(50):
            main()
    except KeyboardInterrupt:
        logging.info(' - Exited by user.')
