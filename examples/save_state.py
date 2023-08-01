#!/usr/bin/env python3
import timeit

import numpy as np
from pprint import pprint
import pybullet as pb
from rbw import shapes, worlds, simulation

ramp = worlds.ramp.default_ramp()
debug = False

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
client = simulation.init_client(debug = debug) # start a server
sim = simulation.init_sim(simulation.RampSim, ramp_data, client) # load ramp into client

#get to ckpt_1
pla, rot, col = simulation.run_trace(sim, debug = debug)
ckpt_1 = (pla[-1], rot[-1])
state_id = pb.saveState(client) # Save the current state, to use in code_new_api

def code_old_api():
    # pla (steps x 3 x objects x 3) : position, linear, angular vel
    # rot (steps x objects x 4) : quaternions
    # col (steps x objects) : collision count
    pla, rot, col = simulation.run_trace(sim, prev_state=ckpt_1, debug = debug) # run simulation starting at ckpt_1
    ckpt_2 = (pla[-1], rot[-1])

    # Create new configuration with updated mass for object A
    new_world = {'A' : {'mass' : 3}} # Update mass to 3 (or desired value)
    simulation.update_world(sim, new_world)

    # Update mass and run the second trace 
    ckpt_3 = (pla[-1], rot[-1])
    print(ckpt_3)

    # Find positions at the last timestep
    pos_ckpt2 = ckpt_2[0][-1, 0, 0]
    pos_ckpt3 = ckpt_3[0][-1, 0, 1]
    pos_ckpt2_objectA = ckpt_2[0][-1, 0]
    print(f"Old API object A position at ckpt_2: {pos_ckpt2_objectA}")
    pos_ckpt3_objectA = ckpt_3[0][-1, 1]
    print(f"Old API object A position at ckpt_3: {pos_ckpt3_objectA}")


def code_new_api():
    #return to ckpt_1
    pb.restoreState(client, str(state_id))
    ckpt_2 = simulation.run_trace(sim, debug = debug)

    # Create new configuration with updated mass for object A
    new_world = {'A' : {'mass' : 3}} # Update mass to 3 (or desired value)
    simulation.update_world(sim, new_world)

    # Update mass and run the second trace
    ckpt_3 = simulation.run_trace(sim, debug=debug)

    # Find positions at the last timestep
    pos_ckpt2 = ckpt_2[0][-1, 0, 0]
    pos_ckpt3 = ckpt_3[0][-1, 0, 1]
    pos_ckpt2_objectA = ckpt_2[0][-1, 0]
    print(f"New API Position at ckpt_2: {pos_ckpt2_objectA}")
    pos_ckpt3_objectA = ckpt_3[0][-1, 1]
    print(f"New API Position at ckpt_3: {pos_ckpt3_objectA}")

#how fast old API is vs saveState
time_old_api = timeit.timeit(code_old_api, number=10)
time_new_api = timeit.timeit(code_new_api, number=10)
print(f"Time for old API: {time_old_api}")
print(f"Time for new API: {time_new_api}")

# Clean up simulator environment
simulation.clear_sim(sim)










