#!/usr/bin/env python3

from pprint import pprint
import numpy as np
from rbw import shapes, worlds, simulation


scene = worlds.marble_box.default_box()

appearance = "Wood" # Wood | Brick | Iron
dims = [0.1, 0.1, 0.1] # in meters
phys_params = {
    'density' : 2.0,
    'lateralFriction' : 0.3,
    'restitution' : 0.1
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
scene.add_object(str(1), ball, 0.5, 0.0)
init_state = np.zeros((4, 1, 3))
# add some initial velocity in x direction
init_state[3, 0] = [0.5, 0, 0]

# run physics
scene_data = scene.serialize() # data must be serialized into a `Dict`
pprint(scene_data)
client = simulation.init_client(debug = True) # start a server
sim = simulation.init_sim(simulation.MarbleSim, scene_data, client) # load ramp into client
print(sim)

# apply state to world
simulation.apply_state(sim, ["0"], init_state)

trace, cols = simulation.run_full_trace(sim, debug = True) # run simulation
print(trace.shape)
simulation.clear_sim(client)
