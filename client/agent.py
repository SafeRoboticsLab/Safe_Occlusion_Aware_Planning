import numpy as np
from client.client import SynchronousClient
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
import carla




class Agent():
    """
    simple constant velocity object
    """
    def __init__(self, client: SynchronousClient, path, s_0, v, filter = 'vehicle.carlamotors.carlacola', init_static = True):
        self.client = client
        self.path = path
        self.init_waypoint = self.client.map.get_waypoint_xodr(path[0][0],path[0][2], s_0)
        self.filter = filter
        self.actor = self.client.spawn_objects(filter, self.init_waypoint)
        self.client.tick() # make sure the object spawn 
        self.agent = BasicAgent(self.actor, target_speed = v*3.6) # target v in km/h
        
        destination = self.client.map.get_waypoint_xodr(path[-1][0],path[-1][2], 3).next_until_lane_end(0.5)[-1]
        #print(destination.road_id, destination.lane_id, destination.s)
        self.agent.set_destination((destination.transform.location.x,
                                   destination.transform.location.y,
                                   destination.transform.location.z))

        self.client.traffic_manager.ignore_lights_percentage(self.actor,100)
        self.client.traffic_manager.distance_to_leading_vehicle(self.actor,0)

        if not init_static:
            R =np.array(self.init_waypoint.transform.get_matrix())[:3, :3]
            target_velocity = np.dot(R, np.array([v*1.1, 0, 0]))
            v = carla.Vector3D(target_velocity[0],target_velocity[1],target_velocity[2])
            self.actor.set_target_velocity(v)

    def tick(self):
        control = self.agent.run_step()
        if 'carlacola' not in self.filter:
            control.throttle = min(control.throttle, 0.5)
        control.manual_gear_shift = False
        self.actor.apply_control(control)

        

    

    


