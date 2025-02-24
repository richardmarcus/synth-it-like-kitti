# Synth It Like Kitti
## Abstract
An important factor in advancing autonomous driving systems is simulation. Yet, there is rather small progress for transferability between the virtual and real world. We revisit this problem for 3D object detection on LiDAR point clouds and propose a dataset generation pipeline based on the CARLA simulator. Utilizing domain randomization strategies and careful modeling, we are able to train an object detector on the synthetic data and demonstrate strong generalization capabilities to the KITTI dataset. Furthermore, we compare different virtual sensor variants to gather insights, which sensor attributes can be responsible for the prevalent domain gap. Finally, fine-tuning with a small portion of real data almost matches the baseline and with the full training set slightly surpasses it.

[Paper](https://arxiv.org/abs/2502.15076)

Citation (Preprint)
```
@misc{marcus2025synth,
    title={Synth It Like KITTI: Synthetic Data Generation for Object Detection in Driving Scenarios},
    author={Richard Marcus and Christian Vogel and Inga Jatzkowski and Niklas Knoop and Marc Stamminger},
    year={2025},
    eprint={2502.15076},
    archivePrefix={arXiv},
    primaryClass={cs.CV}
} 
```


## A Synthetic LiDAR Dataset for Training 3D Object Detection
We provide a dataset with multiple LiDAR variants and instructions for training/evalation here.

### Data Preparation
Download our KITTI formatted [CARLA dataset](https://zenodo.org/records/14901303).
You will find carla.zip to include ImageSets with different sized splits, you need to use the .txt files of one variant.
The other zips contain the point clouds for the different sensor settings.
One exception is the test_sequence.zip, which gives a reference of a simulation sequence processed by process_carla (still including the raw data).

Select one type of LiDAR simulation and extract the folder to follow the [OpenPCDet format](https://github.com/open-mmlab/OpenPCDet/blob/master/docs/GETTING_STARTED.md): 

```
OpenPCDet
├── data
│   ├── kitti
│   │   │── ImageSets
│   │   │── training
│   │   │   ├──calib & velodyne & label_2 & image_2 & (optional: planes) & (optional: depth_2)
│   │   │── testing
│   │   │   ├──calib & velodyne & image_2
├── pcdet
├── tools
```

### Configuring the Training
(OpenPCDet)[https://github.com/open-mmlab/OpenPCDet] is a good starting point for training and evaluating different 3D object detection models, but other code bases that work with KITTI data should also be compatible. 
We have used Voxel R-CNN in our paper and you can use the checkpoint from the model zoo to evaluate on the synthetic dataset.
Alternatively, you can train on the synthetic data from scratch or finetune the checkpoint.

## Creating Your Own Dataset 
We also provide scripts to generate datasets like ours.
This requires building CARLA yourself. (The vanilla binaries are only compatible if you remove the enhanced lidar sensons from our python script.) 

### Custom CARLA installation
Get our custom CARLA 0.9.15. based [fork with enhanced LiDAR implementation.](https://github.com/richardmarcus/carla_lidar/tree/develop)
Follow instructions in readme for building CARLA, note that a special fork of UE4 is required

Optionally, start the editor via ```make launch``` and add additional maps to the UE4 configuration.
Instead of launching, use ```make package```.


### Data Generation
Make sure that the map selection matches those that CARLA is built with.

#### Manual Start
1. ```make package``` generates something like Dist/CARLA_Shipping_0.9.15-1-g614d4338c/LinuxNoEditor/CarlaUE4.sh
You can start this (optionally with -RenderOffScreen) or press play in the editor after ```make launch```.
2. Run ```synthetic_dataset_generator/carla_save_cam_lidar.py --save-data```

#### Automatic Continuus Data Collection
Run ```synthetic_dataset_generator/stable_simulate.sh <path_to_CarlaUE4.sh> <path_to_carla_save_cam_lidar.py> ```

### Data Processing
1. Run ```process_carla.py --base_path <carla_output_folder>```
2. Run ```merge_sequences.py --source_path <carla_output_folder> --destination_path <folder_to_store_final_dataset```
3. (Create ImageSets folder, which contains .txt files for the training, validation and testing splits. You can use custom_image_sets.py as reference)




