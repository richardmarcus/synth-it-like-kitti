import os
import shutil


#read in args
import argparse
parser = argparse.ArgumentParser(description='Merge sequences')
#max number of files to copy
parser.add_argument('--max_files', type=int, default=99999, help='Max number of files to copy')
parser.add_argument('--max_sequences', type=int, default=1000, help='Max number of sequences to copy')
parser.add_argument('--source_folder', type=str, help='Source folder')
parser.add_argument('--destination_folder', type=str, help='Destination folder')

args=parser.parse_args()
# Define the source and destination folders

source_folder = args.source_folder
destination_folder = args.destination_folder

#if destination folder exists, delete it
if os.path.exists(destination_folder):
    shutil.rmtree(destination_folder)

#make the destination folder if it does not exist
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

# Get a list of all folders in the source folder
folders = os.listdir(source_folder)
#sort the folders
folders.sort()
fi = 0
# Iterate over each folder
#velodyne_depth/', 'velodyne_opaque', 'velodyne_dual','velodyne_intensity'
data_folders = ['image_2/','velodyne_front/', 'velodyne_dual/', 'calib/', 'label_2_filtered/', 'label_2/', 'velodyne_intensity/', 'velodyne_opaque', 'velodyne_drop', 'velodyne_depth/', 'velodyne_jitter/']


k=0
for base_folder in folders:
    k+=1
    if k>args.max_sequences:
        break
    #if Town15 in the folder name, skip it
    #if 'Town15' in base_folder:
    #    print(f'Skipping {base_folder}')
    #    continue
    # Get the folder number
    folder_number = fi
    town_name = base_folder.split('_')
    #find the split with Town in it
    for i in range(len(town_name)):
        if 'Town' in town_name[i]:
            town_name= town_name[i]
            break
    
    for data_folder in data_folders:
        max_files = args.max_files
    # Get a list of all files in the current folder
        folder = os.path.join(base_folder, data_folder)
        files = os.listdir(os.path.join(source_folder, folder))
        #sort the files
        files.sort()
        print (f'Processing {folder}')
        
        # Iterate over each file
        for file in files:
            # Add the folder number prefix to the filename
            new_filename = f'{folder_number}_{town_name}_{file}'
            # Get the source and destination paths
            source_path = os.path.join(source_folder, folder, file)
            destination_path = os.path.join(destination_folder,data_folder) 
            if 'velodyne' in source_path:
                #remove /training/ from the path 
                destination_path = destination_path.replace('training/', '/')
                destination_path = destination_path.replace('velodyne_', '')
                #add training at the end
                destination_path = destination_path + '/training/velodyne/'

            destination_path = destination_path.replace('image_0', 'image_2') 
            destination_path = destination_path.replace('label_2/','label_2_og/')
            destination_path = destination_path.replace('label_2_filtered', 'label_2')

            #make the destination folder if it does not exist
            if not os.path.exists(destination_path):
                os.makedirs(destination_path)
            destination_path = os.path.join(destination_path, new_filename)
            
            # Copy the file to the destination folder with the new filename
            #make sure that the file does not exist
            if os.path.exists(destination_path):
                print(f'{destination_path} already exists')
                shutil.copyfile(source_path, destination_path)
            else:
                shutil.copyfile(source_path, destination_path)

            print(f'Copied {source_path} to {destination_path}')
            max_files -= 1
            if max_files == 0:
                print('Max files reached')
                break
        
    fi+=1
    #exit()