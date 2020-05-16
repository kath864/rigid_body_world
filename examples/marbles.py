#!/usr/bin/env python3

from pprint import pprint
import numpy as np
from rbw import shapes, worlds, simulation


scene = worlds.marble_box.default_box()

appearance = "Wood" # Wood | Brick | Iron
dims = [0.1, 0.1, 0.1] # in meters
phys_params = {
    'density' : 2.0,
    'lateralFriction' : 0.3
}

x_pos = np.linspace(0.4, 1.6, 3)
y_pos = np.linspace(0.4, 1.6, 3)
c = 0
for xy in zip(*np.meshgrid(x_pos, y_pos)):
    ball = shapes.Ball(appearance, dims, phys_params)
    scene.add_object(str(c), ball, xy)
    c += 1

# run physics
scene_data = scene.serialize() # data must be serialized into a `Dict`
pprint(scene_data)
client = simulation.init_client(debug = True) # start a server
sim = simulation.init_sim(simulation.MarbleSim, scene_data, client) # load ramp into client

trace, cols = simulation.run_full_trace(client, sim, debug = True) # run simulation
print(trace.shape)
simulation.clear_trace(client)
