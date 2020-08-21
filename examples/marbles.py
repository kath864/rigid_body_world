#!/usr/bin/env python3

from pprint import pprint
import numpy as np
from rbw import shapes, worlds, simulation
from rbw.fields import BeamField


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
# add balls in a grid
# for x in np.linspace(-0.6, 0.6, 3):
#     for y in np.linspace(-0.6, 0.6, 3):
#         ball = shapes.Ball(appearance, dims, phys_params)
#         scene.add_object(str(c), ball, x, y)
#         c += 1x, y

# add a single ball to the middle
ball = shapes.Ball(appearance, dims, phys_params)
scene.add_object(str(c), ball, 0.0, 0.0)
beam = BeamField(str(c), force = [1,0,0])
scene.add_field('beam', beam)
                 # force = [1, 0, 0])
wall = shapes.Ball(appearance, dims, dict(density = 100.,
                                           lateralFriction = 0.9,
                                           restitution = 0.9))
scene.add_object(str(1), wall, 1, 0.0)

# run physics
# scene_data = scene.serialize(indent = 2) # data must be serialized into a `Dict`
# client = simulation.init_client(debug = False) # start a server
client = simulation.init_client(debug = True) # start a server
states = simulation.run_full_trace(client, scene.graph, T = 5)
kps = simulation.keypoints(states)
print(list(scene.graph.nodes))
pprint(kps)

# sim = simulation.init_sim(simulation.MarbleSim, scene_data, client) # load ramp into client
# print(sim)

# pla, rot, col = simulation.run_full_trace(sim, debug = True) # run simulation
# print(col)
# # # print(trace.shape)
# simulation.clear_sim(sim)
