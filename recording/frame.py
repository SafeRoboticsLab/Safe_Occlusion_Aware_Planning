from planning import HybridGame
import carla

class Frame:
    def __init__(self, game: HybridGame):
        self.t = game.t_cur
        self.mode = game.open_loop

        # record actors
        self.ego = Object(game.client.ego)
        self.ego_id = game.client.ego.id

        self.actor_list = []
        actor_list = game.client.world.get_actors()
        for actor in actor_list:
            if actor.id != self.ego_id:
                self.actor_list.append(Object(actor))

        # record infospace
        self.shadow = game.info_space.shadow_manager.shadow_list


class Object:
    def __init__(self, actor):
        self.id = actor.id
        self.filter = actor.type_id

        transform = actor.get_transform()
        location = transform.location
        rotation = transform.rotation
        self.location = [location.x, location.y, location.z] #[x,y,z,roll,pitch,yaw]
        self.rotation = [rotation.roll, rotation.pitch, rotation.yaw]

        velocity = actor.get_velocity()
        self.velocity = [velocity.x, velocity.y, velocity.z]
    

