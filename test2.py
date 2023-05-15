import pickle
import math
from test import *
with open('array.pickle', 'rb') as f:
    # Load the contents of the file into a Python object
    data = pickle.load(f)

    # for ts in data:
    #     for obj in ts.objects:
    #         # if(obj.id == ("7567505")):
    #         if(obj.id == ("41867819")):
    #             print(obj.rotation, math.degrees(obj.rotation))

process_pickle()
make_images()
count = 0

for ts in data:

    idx = 0
    for obj in ts.objects:
        if obj.id != "41867819":
            continue
        frame_to_image_wrt_object(ts, idx, count)
        idx+=1
    count+=1

path = "test"

for name in os.listdir(path):

    image_folder = "test/" + "41867819"
    image_format = 'png'
    print(image_folder)
    
    # Set duration for each frame in milliseconds
    frame_duration = 200

    # Create list of image file paths
    image_paths = sorted([os.path.join(image_folder, fn) for fn in os.listdir(image_folder) if fn.endswith('.png')])
    # Open each image and add to frames list
    frames = []
    for image_path in image_paths:
        with Image.open(image_path) as im:
            frames.append(im.copy())
    # print(len(frames))

    # Save frames to", image_folder+"/animation.gif")
    frames[0].save(f"{image_folder}/animation.gif", format='GIF', append_images=frames[1:], save_all=True, duration=frame_duration, loop=0)