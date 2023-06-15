import pickle
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation


def visualize_trajectories(pickle_file):
    # load the pickle file
    with open(pickle_file, 'rb') as f:
        trajectories = pickle.load(f)
        # trajectories is a dictionary mapping object id to a list of tuples
        # of the form (class, x, y, speed, angle)
        # create a plot
        fig = plt.figure()
        # update the data for each line
        for obj_id in trajectories:
            x = [trajectories[obj_id][i][1]
                 for i in range(len(trajectories[obj_id]))]
            y = [trajectories[obj_id][i][2]
                 for i in range(len(trajectories[obj_id]))]
            plt.plot(x, y)
        # set legend with object ids and title for legend
        plt.legend(trajectories.keys(), title="Object ID")
        # show the plot
        plt.show()


visualize_trajectories("trajectory.pickle")
