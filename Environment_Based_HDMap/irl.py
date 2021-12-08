import time

from envs.merge_env import *
from envs.common.graphics import EnvViewer

file_name = "DR_CHN_Merging_ZS0"
osm_file_path = "/home/joe/Prediction/Dataset/interaction-dataset/maps/"+file_name+".osm"

file_path = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/train/"+file_name+"_train.csv"

env = MergeEnv(dataset_file_path=file_path, osm_file_path=osm_file_path, vehicle_id=5, case_id=1)
env.reset(reset_time=1)

length = env.vehicle.interaction_traj.shape[0]

# Data collection
buffer = []
human_traj_features = []
timesteps = np.linspace(2, length-10, num=5, dtype=np.int16)
train_steps = np.random.choice(timesteps, size=4, replace=False)
test_steps = [t for t in timesteps if t not in train_steps]

for start in train_steps:
    env.reset(reset_time=start)

    lateral_offsets, target_speeds = env.sampling_space()

    # set up buffer of the scene
    buffer_scene = []

    # lateral and speed trajectory sampling
    print('scene: {}, sampling...'.format(start))
    for lateral in lateral_offsets:
        for target_speed in target_speeds:
            # sample a trajectory
            action = (lateral, target_speed, 1)
            obs, features, terminated, info = env.step(action)

            # render env
            if True:
                env.render()

            # get the features
            traj_features = features[:-1]
            human_likeness = features[-1]

            # add the trajectory to scene buffer
            buffer_scene.append([lateral, target_speed, traj_features, human_likeness])

            # go back to original scene
            env.reset(reset_time=start)

    # calculate human trajectory feature
    env.reset(reset_time=start, human=True)
    obs, features, terminated, info = env.step()

    # eliminate invalid examples
    if terminated or features[-1] > 2.5:
        continue

    # process data
    human_traj = features[:-1]
    buffer_scene.append([0, 0, features[:-1], features[-1]])

    # add generated and human trajectories to buffer
    human_traj_features.append(human_traj)
    buffer.append(buffer_scene)
