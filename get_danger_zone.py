from recording import RecordUtil
from scipy.io import savemat
import os
import numpy as np
from client import SynchronousClient, HUD, PraseArgs, Frame, Agent
from risk_analysis import ISpace

args = PraseArgs()
args.map = 'Town01'

#filename = 'interseciton_close_no_obj'
#filename = 'interseciton_baseline_no_obj'
#filename = 'interseciton_close_obj_pass'
filename = 'interseciton_close_obj_yield'

#filename = 'overtake_baseline_no_obj'
#filename = 'overtake_close_no_obj'
#filename = 'overtake_close_obj_evade_before_overtake'
#filename = 'overtake_close_obj_evade_after_overtake'

record_folder = os.getcwd()+ "/results/Good/Intersection/"
record_filename = record_folder+filename+".pkl"

i = 143

save_to =  '/hdd/GoogleDrive/Research/Safe Active Perception/Figures/Results/Intersection/'+filename+'_'+str(i)+'.mat'


client = SynchronousClient(args)
info_space = ISpace(client, None, [])

record = RecordUtil()
record.load_from_file(record_filename)

num_frame = record.num_frame


# ego's information
traj = np.zeros((7, num_frame))

frame = record.record.frame_list[i]

mdic = {}

for (i, danger_zone) in enumerate(frame.danger_zone):
    for j, polygon in enumerate(danger_zone.coverage_list):
        name = "danger_"+str(i)+'_'+str(j)
        array = np.array([polygon[0], polygon[1]])
        mdic[name] = array

shadow_plot = info_space.get_shadow_for_plot(frame.shadow)

for i, shadow in enumerate(shadow_plot):
    name = "shadow_"+str(i)
    array = np.array([shadow[0], shadow[1]])
    mdic[name] = array



#print(mdic)

savemat(save_to, mdic)
client.destroy()



