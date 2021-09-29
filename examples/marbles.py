#!/usr/bin/env python3

from pprint import pprint
import numpy as np
from rbw.shapes import Ball
from rbw.worlds import MarbleWorld
from rbw import simulation
from rbw.utils.render import render
from rbw.utils.ffmpeg import continuous_movie

# first define world geometry
scene = MarbleWorld([3., 3., 0.1], # dimensions of room in meters
                    # physical properties of table top
                    table_phys = {
                        'density' : 0.0,
                        'lateralFriction' : 0.3,
                        'restitution' : 0.9
                    })

# appearance as defined as a texture in marble.blend
appearance = "wood"
dims = [0.1, 0.1, 0.1] # in meters

# for a complete list of all physical properties take a look at the pybullet quickstart guide
# under `changeDynamics`
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
phys_params = {
    'density' : 2.0,
    'lateralFriction' : 0.3,
    'restitution' : 0.9
}

# add a single ball to the middle
ball_a = Ball(appearance, dims, phys_params)
scene.add_object('a', ball_a,
                 0.0, 0.0, # x,y coordinates
                 force = [2.5, 0, 0]) # impart an instantaneous force along x-axis

# ball a will collide with ball b
ball_b = Ball(appearance, dims, phys_params)
scene.add_object('b', ball_b,
                 0.5, 0.0) # xy coordinates

# run physics
scene_data = scene.serialize() # data must be serialized into a `dict`
pprint(scene_data)
client = simulation.init_client(debug = False) # start a server
sim = simulation.init_sim(simulation.MarbleSim, scene_data, client) # load ramp into client
pla, rot, col = simulation.run_full_trace(sim, T = 1.0) # run simulation for 2.5s
# pla (steps x 3 x objects x 3) : position, linear, angular vel
# rot (steps x objects x 4) : quaternions
# col (steps x objects) : collision count
simulation.clear_sim(sim)

pprint(pla[:, 0, 0])

# Render
kwargs = dict(
    # scene data and trace
    scene = {'scene': scene_data},
    # packaging for rendering
    # rendering expects positions and orientations for each object
    trace = {'pos' : pla[:, 0, :], 'orn' : rot},
    out = './marble_render', # destination of rendering
    # rendering enviroment
    exec = 'blender', # blender executable
    # blend source file
    # can contain an arbitrary amount of context including:
    # - rendering parameters (rendering engine, resolution...)
    # - background, skybox
    # - lighting
    # - camera
    # Note that most of these features can also be described in
    # the `render` script
    blend = './marbles.blend',
    render = './render.py', # blender bpy script
    render_mode = 'default', # argument for script
    resolution = (512, 512)
)
# will result in a folder `marble_render`
# that will have a blend file of the scene
# as well as a folder with frames rendered using Blender's cycles engine
render(**kwargs)


# You could use tools like ffmpeg to create videos from these
# here we're using a wrapper over ffmpeg
continuous_movie('./marble_render/render/%d.png', # src
                 './marbles') # dst, will append ".mp4"
