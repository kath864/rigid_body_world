#!/usr/bin/env python3

from pprint import pprint

from rbw import shapes, worlds, simulation


ramp = worlds.ramp.default_ramp()

appearance = "Wood" # Wood | Brick | Iron
dims = [0.3, 0.3, 0.3] # in meters
phys_params = {
    'density' : 2.0,
    'lateralFriction' : 0.3
}
ramp_obj = shapes.Block(appearance, dims, phys_params)
table_obj = shapes.Puck(appearance, dims, phys_params)

ramp.add_object('A', ramp_obj, 1.7)
ramp.add_object('B', table_obj, 0.3)

# run physics
ramp_data = ramp.serialize() # data must be serialized into a `Dict`
pprint(ramp_data)
client = simulation.init_client(debug = True) # start a server
sim = simulation.init_sim(simulation.RampSim, ramp_data, client) # load ramp into client

trace, cols = simulation.run_full_trace(client, sim, debug = True) # run simulation
print(trace.shape)
simulation.clear_trace(client)
