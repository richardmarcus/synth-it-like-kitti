# CARLA synthetic dataset creation

Start CARLA simulator.

Run `carla_save_cam_lidar.py --save-data` to save Camera, Depth, and Lidar data together with calibration files and label files for Bounding Boxes in the KITTI style.

The script will save data to an output folder and will delete the data from the previous run (if any data exists). Then it will create the folder structure according to KITTI and save the generated data.
For the run it will spawn a number of vehicles and pedestrians, parking vehicles and props on the side.

Settings like the map or the number of vehicles, walkers can be adjusted in the settings part of the script.

Data generation has the following settings:

- Adapted KITTI sensor set and mounting position (2 rgb, 2 grayscale, LiDAR and additional depth camera in LiDAR perspective).
  - See for <https://www.cvlibs.net/datasets/kitti/setup.php> details
- Some approximation had to be done, because the KITTI vehicle (VW Passat B6) is not available as a CARLA model.
- Data will be saved in the same way KITTI dataset does (names, folder structure etc.)
  - KITTI calibration file does not change, but will be saved each recording step nonetheless
- Saving to disk can be switched on and off
  - But data will only be saved if
    - vehicle is moving (speed > 0.1 m/s)
    - at least 1 bounding box inside camera frame
    - only between an adjustable delta simulation step

The following flags are available for running the script:

- ```--host```: Host-IP of Carla server, default is ```localhost```
- ```--port```: Port of Carla server, default is ```2000```
- ```--save-data```: Save KITTI like data to disk (if not set, no data will be saved)
- ```--record```: Record simulation run using Carla's internal recording function
- ```--record-file```: Name and path of recording file, using \, / or : characters in the file name will define it as an absolute path, default uses Carla's default location
- ```--replay```: Replay simulation run using Carla's internal replay function
- ```--record-file```: Name and path of replay file. Using \, / or : characters in the file name will define it as an absolute path, default uses Carla's default location

To resume recording after a crash you have to comment out the generation of the output folders (line 560 - 573) or previously recorded data will be deleted.
You also have to set the start frame id to the next frame that shall be recorded, line 530: frame_id = 0 (e.g. last frame recorded before crash was 345, set frame_id to 346).
