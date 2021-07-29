from planning import HybridGame
from recording.frame import Frame, Object
import random
import carla


class Record():
    def __init__(self, map_name):
        # a set of parameters
        self.map_name = map_name
        self.frame_list = []
        self.active_actors = {}

    def add_frame(self, game: HybridGame):
        self.frame_list.append(Frame(game))

    def replay_frame(self, idx, client):
        if idx < len(self.frame_list):
            cur_frame = self.frame_list[idx]
            self.spawn_object(cur_frame.ego)
            for actor in cur_frame.actor_list:
                self.spawn_object(actor, client)

    def spawn_object(self, actor:Object, client):
        transform = carla.Transform(carla.Location(x=actor.location[0], y=actor.location[1], z=actor.location[2]),
                            carla.Rotation(roll=actor.rotation[0], pitch=actor.rotation[1], yaw=actor.rotation[2]))
        if actor.id not in self.active_actors:
            ego_bp = random.choice(client.blueprint_library.filter(actor.filter))
            client_actor = client.world.spawn_actor(ego_bp, transform)
            self.active_actors[actor.id] = client_actor.id
        else:
            client_actor_id = self.active_actors[actor.id]
            client_actor = client.world.get_actor(client_actor_id)
            client_actor.set_transform(transform)




