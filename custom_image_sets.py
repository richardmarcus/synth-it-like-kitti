import os
import random
import argparse

parser = argparse.ArgumentParser(description='Create custom image sets for training and validation.')
parser.add_argument('--imageset_path', type=str,help='path to ImageSets folder')
parser.add_argument('--label_path', type=str,help='Path to label folder')

args = parser.parse_args()

base_set_path = args.imageset_path
data_path = args.label_path



set_path = os.path.join(base_set_path, "Big/")
set_path_medium = os.path.join(base_set_path, "Medium/")
set_path_small = os.path.join(base_set_path, "Small/")
set_path_tiny = os.path.join(base_set_path, "Tiny/")

if not os.path.exists(set_path):
    os.makedirs(set_path)
if not os.path.exists(set_path_medium):
    os.makedirs(set_path_medium)
if not os.path.exists(set_path_small):
    os.makedirs(set_path_small)
if not os.path.exists(set_path_tiny):
    os.makedirs(set_path_tiny)


#filename structure: 0_Town04_000000.txt

files = os.listdir(data_path)
#sort
files.sort()
towns = []
sequences = []
sequence_begins = []    

i = 0
for file in files:
    town = file.split('_')[1]
    sequence = file.split('_')[0]
    if town not in towns:
        towns.append(town)

    if sequence not in sequences:
        sequences.append(sequence)
        sequence_begins.append(i)
    i+=1
sequence_begins.append(len(files))


   

#pick random 50% of sequences

#get random indices
indices = range(len(sequences))
#get actual indices
indices = [i for i in indices]
#shuffle



random.shuffle(indices)


train_indices = indices[:int(len(indices)*0.5)]
test_indices = indices[int(len(indices)*0.5):]

train_files = []
test_files = []
for i in train_indices:
   for j in range(sequence_begins[i], sequence_begins[i+1]):
       train_files.append(files[j][:-4])

for i in test_indices:
   for j in range(sequence_begins[i], sequence_begins[i+1]):
       test_files.append(files[j][:-4])


#shuffle
random.shuffle(train_files)
random.shuffle(test_files)

train_path = set_path + 'train.txt'
test_path = set_path + 'val.txt'


train_small_path = set_path_small + 'train.txt'
test_small_path = set_path_small + 'val.txt'

train_medium_path = set_path_medium + 'train.txt'
test_medium_path = set_path_medium + 'val.txt'

train_tiny_path = set_path_tiny + 'train.txt'
test_tiny_path = set_path_tiny + 'val.txt'

actual_test_path = set_path_medium + 'test.txt'

medium_train_files = train_files[:int(len(train_files)*0.5)]
medium_test_files = test_files[:int(len(test_files)*0.5)]

medium_actual_test_files = test_files[int(len(test_files)*0.5):]

small_train_files = train_files[:int(len(medium_train_files)*0.5)]
small_test_files = test_files[:int(len(medium_test_files)*0.5)]

tiny_train_files = train_files[:int(len(small_train_files)*0.1)]
tiny_test_files = test_files[:int(len(small_test_files)*0.1)]



with open(actual_test_path, 'w') as f:
    for item in medium_actual_test_files:
        f.write("%s\n" % item)

with open(train_path, 'w') as f:
    for item in train_files:
        f.write("%s\n" % item)

with open(test_path, 'w') as f:
    for item in test_files:
        f.write("%s\n" % item)

with open(train_small_path, 'w') as f:
    for item in small_train_files:
        f.write("%s\n" % item)

with open(test_small_path, 'w') as f:
    for item in small_test_files:
        f.write("%s\n" % item)

with open(train_medium_path, 'w') as f:
    for item in medium_train_files:
        f.write("%s\n" % item)

with open(test_medium_path, 'w') as f:
    for item in medium_test_files:
        f.write("%s\n" % item)

with open(train_tiny_path, 'w') as f:
    for item in tiny_train_files:
        f.write("%s\n" % item)

with open(test_tiny_path, 'w') as f:
    for item in tiny_test_files:
        f.write("%s\n" % item)






