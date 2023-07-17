#!/usr/bin/env python3

from pprint import pprint
import pybullet as pb

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

debug = True
# run physics
ramp_data = ramp.serialize() # data must be serialized into a `Dict`
pprint(ramp_data)
client = simulation.init_client(debug = debug) # start a server
sim = simulation.init_sim(simulation.RampSim, ramp_data, client) # load ramp into client

# pal (steps x 3 x objects x 3) : position, linear, angular vel
# rot (steps x objects x 4) : quaternions
# col (steps x objects) : collision count
pal, rot, col = simulation.run_full_trace(sim, debug = debug, T=2.0) # run simulation
ckpt_1 = (pal[-1], rot[-1])

# Save the state of the simulation
state_id = pb.saveState(client)  # Save the current state
print(f"state ID is {state_id}")
pprint(sim)
ckpt_2 = simulation.run_trace(sim, prev_state = ckpt_1, debug = True)

# Create new configuration with updated mass for object A
new_world = {'A' : {'mass' : 3}} # Update mass to 3 (or desired value)
simulation.update_world(sim, new_world)

# Update mass and run the second trace
simulation.update_world(sim, new_world)
ckpt_3 = simulation.run_trace(sim, prev_state=ckpt_1, debug=True)

# Compare ckpt_2 and ckpt_3
if ckpt_2 == ckpt_3:
    print("ckpt_2 and ckpt_3 are identical")
else:
    print("ckpt_2 and ckpt_3 differ")

# More specific comparison
for i in range(len(ckpt_2)):
    if ckpt_2[i] != ckpt_3[i]:
        print(f"ckpt_2 and ckpt_3 differ at index {i}: {ckpt_2[i]} vs {ckpt_3[i]}")

# Run a trace using ckpt_1 and the updated world configuration
ckpt_3 = simulation.run_trace(sim, prev_state=ckpt_1, debug=True)
pprint(ckpt_3)

# Restore the state of the simulation
pb.restoreState(client, state_id)

# Clean up simulator environment
simulation.clear_sim(sim)









