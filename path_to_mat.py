from recording import RecordUtil
from scipy.io import savemat
import os
import numpy as np

#filename = 'interseciton_close_no_obj'
#filename = 'interseciton_baseline_no_obj'
#filename = 'interseciton_close_obj_pass'
#filename = 'interseciton_close_obj_yield'
filename = 'blind_summit_baseline_no_obj'
#filename = 'overtake_baseline_no_obj'
#filename = 'overtake_close_no_obj'
#filename = 'overtake_close_obj_evade_before_overtake'
#filename = 'overtake_close_obj_evade_after_overtake'
record_folder = os.getcwd()+ "/results/planning/"
record_filename = record_folder+filename+".pkl"

save_to =  '/hdd/GoogleDrive/Research/Safe Active Perception/Figures/Results/blind_summit/'+filename+'.mat'


record = RecordUtil()
record.load_from_file(record_filename)

num_frame = record.num_frame


# ego's information
traj = np.zeros((7, num_frame))

for i, frame in enumerate(record.record.frame_list):
    traj[6,i] = frame.t
    location = frame.ego.location
    progress = frame.progress
    traj[0,i] = location[0]
    traj[1,i] = location[1]
    traj[2,i] = location[2]

    traj[3,i] = progress[0]
    traj[4,i] = progress[1]
    traj[5,i] = progress[2]

mdic = {'ego_close_before': traj}


# get obj list
traj_map = {}
for i, frame in enumerate(record.record.frame_list):
    for obj in frame.actor_list:
        obj_id = obj.id
        if obj_id not in traj_map:
            cur_traj = np.zeros((4, num_frame))
        else:
            cur_traj = traj_map[obj_id]

        cur_traj[3,i] = frame.t
        location = obj.location
        cur_traj[0,i] = location[0]
        cur_traj[1,i] = location[1]
        cur_traj[2,i] = location[2]
        traj_map[obj_id] = cur_traj
        
for i, pair in enumerate(traj_map.items()):
    id = 'obj'+str(i+1)
    mdic[id] = pair[1]

#print(mdic)

savemat(save_to, mdic)



