import os


base_path = "/home/oq55olys/Projects/carla/carla-datasets/synthetic_dataset_generator/data/kitti/training"
merged_path = "/home/oq55olys/Projects/carla/carla-datasets/synthetic_dataset_generator/data/kitti/merged"
if not os.path.exists(merged_path):
    os.makedirs(merged_path)
paths = os.listdir(base_path)
#sort
paths.sort()

for path in paths:

    image_folder = os.path.join(base_path, path, "image_2/")
    for image in os.listdir(image_folder):
        image_path = os.path.join(image_folder, image)
        image = path + "_" + image
        print("copied from", image_path, "to", os.path.join(merged_path, image))
        os.system("cp " + image_path + " " + os.path.join(merged_path, image))
