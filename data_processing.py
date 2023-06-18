import numpy as np
import pickle
import pandas as pd
import argparse


def prepare_interaction_matrix(object_ids, pickle_file):
    data = None
    # Load the pickle file
    with open(pickle_file, "rb") as f:
        # Load the contents of the file into a Python object
        data = pickle.load(f)
        # data is a nested dictionary mapping object id to a dictionary of the form
        # {'frame': frame_number, 'class': class_type, 'x': x_coordinate, 'y': y_coordinate, 'speed': speed}
        
    if data is None:
        print("No data to process")
        return
    # ensure the object ids are in the data
    for obj_id in object_ids:
        if obj_id not in data:
            print(f"Object {obj_id} not in data")
            return

    # get common frames between all objects
    common_frames = set(data[object_ids[0]]['frames'].keys())
    if len(object_ids) > 1:
        for obj_id in object_ids[1:]:
            common_frames = common_frames.intersection(set(data[obj_id]['frames'].keys()))
            if len(common_frames) == 0:
                print("No common frames between objects")
                return
    common_frames = sorted(list(common_frames))
    # np array will be of the format T x (N(4 + N)) where T is the number of common frames and N is the number of objects
    # every (4 + N) columns will be for one object where the first 4 columns are for the object itself and the next N columns is a one hot encoding of 
    # which object it is
    matrix = np.zeros((len(common_frames), len(object_ids) * (4 + len(object_ids))))
    prev_frame_xy = [(0, 0) for i in range(len(object_ids))]
    # populate the matrix
    for frame_idx, frame in enumerate(common_frames):
        for idx, obj_id in enumerate(object_ids):
            # get the object
            obj = data[obj_id]['frames'][frame]
            # get the index of the object in the matrix
            obj_idx = idx * (4 + len(object_ids))
            # populate the matrix
            matrix[frame, obj_idx] = obj['x']
            matrix[frame, obj_idx + 1] = obj['y']
            matrix[frame, obj_idx + 2] = obj['speed']
            # compute theta based on the previous frame
            if frame_idx != 0:
                matrix[frame, obj_idx + 3] = np.arctan2(obj['y'] - prev_frame_xy[idx][1], obj['x'] - prev_frame_xy[idx][0])
            matrix[frame, obj_idx + 4 + idx] = 1
            # update the previous frame for the correct object
            prev_frame_xy[idx] = (obj['x'], obj['y'])
    # name of the input pickle file
    pickle_file_name = pickle_file.split("/")[-1].split(".")[0]
    # save the matrix
    np.save(f"{pickle_file_name}_interaction_matrix.npy", matrix)
    


if __name__ == "__main__":
    # get input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("pickle_file", help="pickle data file to parse")
    parser.add_argument("object_ids", nargs="+", help="object ids to visualize")
    args = parser.parse_args()
    if args.pickle_file == None or args.object_ids == None:
        print("Usage: python data_processing.py <pickle_file> <object_ids>")
        sys.exit(1)
    # prepare the interaction matrix
    prepare_interaction_matrix(args.object_ids, args.pickle_file)










