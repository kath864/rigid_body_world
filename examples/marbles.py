#!/usr/bin/env python3

from pprint import pprint
import networkx as nx
import numpy as np
from rbw import shapes, worlds, simulation
from rbw.fields import BeamField
import json


scene = worlds.marble_box.MarbleWorld([3, 3],
                                      table_phys = {
                                          'density' : 0.0,
                                          'lateralFriction' : 0.3,
                                          'restitution' : 0.9
                                      })

appearance = "Wood" # Wood | Brick | Iron
dims = [0.1, 0.1, 0.1] # in meters
phys_params = {
    'density' : 2.0,
    'lateralFriction' : 0.3,
    'restitution' : 0.9
}

x_pos = np.linspace(0.4, 1.6, 3)
y_pos = np.linspace(0.4, 1.6, 3)
c = 0
# add a single ball to the middle
ball = shapes.Ball(appearance, dims, phys_params)
ball.linear_velocity = [1, 0, 0]
scene.add_object(str(c), ball, 0.0, 0.0)
beam = BeamField(str(c), force = [1,0,0])

# add a very heavy ball that acts like a wall
wall = shapes.Ball(appearance, dims, dict(density = 100.,
                                           lateralFriction = 0.9,
                                           restitution = 0.9))
scene.add_object(str(1), wall, 1, 0.0)

# run physics
scene_data = scene.serialize(indent = 2) # data must be serialized into a `Dict`
graph = scene.graph # can also parse from str : worlds.parse_world_graph(scene_data)
client = simulation.init_client(debug = False) # start a server
states = simulation.run_full_trace(client, graph, T = 5, debug = False)
kps = simulation.keypoints(states)
pprint(kps)

parsed_state = simulation.parse_state(states, ['0', '1'], 12)
# pprint(parsed_state)

# sim = simulation.init_sim(simulation.MarbleSim, scene_data, client) # load ramp into client
# print(sim)

# pla, rot, col = simulation.run_full_trace(sim, debug = True) # run simulation
# print(col)
# # # print(trace.shape)
# simulation.clear_sim(sim)
