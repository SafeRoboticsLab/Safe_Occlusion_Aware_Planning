from client.client import SynchronousClient
from planning import HybridGame
from recording.frame import Frame, Object
import random
import carla
import pickle



class Record():
    def __init__(self):
        # a set of parameters
        self.frame_list = []

class RecordUtil():
    def __init__(self):
        self.record = Record()
        self.active_actors = {}

    @property
    def num_frame(self):
        return len(self.record.frame_list)

    
    def load_from_file(self, frilename):
        file = open(frilename,'rb')
        self.record = pickle.load(file)
        file.close()

    def save_to_file(self, filename):
        pickle.dump(self.record, open(filename, "wb"))


    def add_frame(self, game: HybridGame):
        self.record.frame_list.append(Frame(game))

    def replay_frame(self, idx, client):
        if idx < len(self.record.frame_list):
            cur_frame = self.record.frame_list[idx]
            self.spawn_object(cur_frame.ego, client, True)
            for actor in cur_frame.actor_list:
                self.spawn_object(actor, client)
            return cur_frame.shadow, cur_frame.danger_zone, cur_frame

    def spawn_object(self, actor:Object, client: SynchronousClient, ego = False):
        z = actor.location[2]
        if z<0.05:
            z = 0.05

        transform = carla.Transform(carla.Location(x=actor.location[0], y=actor.location[1], z=z),
                            carla.Rotation(roll=actor.rotation[0], pitch=actor.rotation[1], yaw=actor.rotation[2]))
        if actor.id not in self.active_actors:
            ego_bp = random.choice(client.blueprint_library.filter(actor.filter))
            client_actor = client.world.spawn_actor(ego_bp, transform)
            self.active_actors[actor.id] = client_actor.id
            if ego: 
                client.ego = client_actor
            else:
                client.vehicle_list.append(client_actor)
        else:
            client_actor_id = self.active_actors[actor.id]
            client_actor = client.world.get_actor(client_actor_id)
            client_actor.set_transform(transform)




