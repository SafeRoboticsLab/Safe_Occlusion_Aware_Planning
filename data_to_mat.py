from recording import RecordUtil
from scipy.io import savemat
import os
import numpy as np

#filename = 'interseciton_close_no_obj'
filename = 'interseciton_baseline_no_obj'
record_folder = os.getcwd()+ "/results/Good/Intersection/"
record_filename = record_folder+filename+".pkl"

save_to =  '/hdd/GoogleDrive/Research/Safe Active Perception/Figures/Results/Intersection/'+filename+'.mat'


record = RecordUtil()
record.load_from_file(record_filename)

num_frame = record.num_frame


# ego's information
x = []
y = []
z = []
p = []
l = []
v = []
t = []
for frame in record.record.frame_list:
    t.append(frame.t)
    location = frame.ego.location
    progress = frame.progress
    x.append(location[0])
    y.append(location[1])
    z.append(location[2])
    p.append(progress[0])
    l.append(progress[1])
    v.append(progress[2])

ego_mat = np.array([x,y,z,p,l,v,t])
mdic = {'ego_open': ego_mat}
savemat(save_to, mdic)



