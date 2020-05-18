""" A collection of functions interfacing with pybullet simulations

Implemented as functions rather than classes to facilitate cross-language support.
"""
from rbw import np
from . import pybullet
from .sim import clean_params

import time
import operator as op
from functools import reduce
from itertools import combinations

#######################################################################
# Interface
#######################################################################

def init_client(debug = False, **kwargs):
    """" Returns an initialized pybullet client. """
    if debug:
        t = pybullet.GUI
    else:
        t = pybullet.DIRECT
    cid = pybullet.connect(t, **kwargs)
    return cid

def init_sim(sim, data, cid):
    """ Loads a world into a client and returns an sim map

    :param sim: A `Sim` class
    :param data: json-serialized scene data to be parsed by `sim`
    :param cid: The client id to using for `sim`
    """
    s = sim(data, cid)
    return s.serialize()

def update_world(sim_map, new_world):
    """ Updates the world's physical parameters

    :param client: Pybullet client id
    :param object_map: The object name -> object id `dict`
    :new_world: A serialized scene containing new physical properties
    """
    client = sim_map['client']
    obj_map = sim_map['world']
    for obj_key, obj_id in obj_map.items():
        if obj_key in new_world:
            params = new_world[obj_key]
            _update_obj(obj_id, params, client)


def apply_state(sim_map, key_order, state):
    """ Applies a state matrix to each object reported

    The expected dimensions of state are obsxSTATE
    where `obs` is the number of objects (in order)
    and STATE is the tuple representing the dimensions of the
    component.

    :param object_ids: A list of ids corresponding objects in `state`
    :param state: A tuple of position, rotation, angular velocity and linear velocity for each object.
    """
    client = sim_map['client']
    (positions, rotations, ang_vel, lin_vel) = state
    obj_map = sim_map['world']
    for i, obj_key in enumerate(key_order):
        ob_id = obj_map[obj_key]
        quat = pybullet.getQuaternionFromEuler(rotations[i])
        pybullet.resetBasePositionAndOrientation(ob_id,
                                                 posObj = positions[i],
                                                 ornObj = quat,
                                                 physicsClientId = client)
        pybullet.resetBaseVelocity(ob_id,
                                   linearVelocity = lin_vel[i],
                                   angularVelocity = ang_vel[i],
                                   physicsClientId = client)

def step_trace(client, sim, dur, time_step = 240, fps = 60,
               time_scale = 1.0, state = None, debug = False,
               ret_col = True):
    """Obtains sim state from simulation.

    Currently returns the position of each rigid body.
    The total duration is equal to `frames / fps`

    Arguments:
        dur (float): The amount of time in seconds to simulate
        objects ([str]): List of strings of objects to report
        time_step (int, optional): Number of physics steps per second
        fps (int, optional): Number of frames to report per second.
    """
    pybullet.setPhysicsEngineParameter(
        enableFileCaching = 0,
        fixedTimeStep = 1/time_step,
        physicsClientId = client
    )

    # Configure time steps
    dt = 1.0 / fps * time_scale
    frames = int(np.floor(dur * fps))

    object_ids = [sim[k] for k in sorted(sim.keys())]
    n_objs = len(object_ids)

    if not state is None:
        apply_state(client, object_ids, state)

    state = np.zeros((frames, 4, n_objs, 3))
    collisions = np.zeros((frames, _ncr(n_objs, 2)))

    if debug:
        pybullet.setRealTimeSimulation(1, physicsClientId = client)
        while (1):
            keys = pybullet.getKeyboardEvents()
            print(keys)
            time.sleep(0.01)
        return


    for frame in range(frames):
        def update_cols():
            for c,(a,b) in enumerate(combinations(object_ids, 2)):
                cp = len(pybullet.getContactPoints(bodyA=a, bodyB=b,
                                                   physicsClientId = client))
                collisions[frame, c] += cp

        _step_simulation(client, dt, time_step, update_cols)

        for c, obj_id in enumerate(object_ids):
            pos, rot = pybullet.getBasePositionAndOrientation(obj_id,
                                                              physicsClientId = client)
            l_vel, a_vel = pybullet.getBaseVelocity(obj_id,
                                                    physicsClientId = client)
            state[frame, 0, c] = pos
            state[frame, 1, c] = pybullet.getEulerFromQuaternion(rot)
            state[frame, 2, c] = a_vel
            state[frame, 3, c] = l_vel

    if ret_col:
        return state, collisions
    return state



def clear_sim(sim_map):
    """Clean up connection if not already done."""
    # print('disconnecting from {0:d}'.format(client))
    client = sim_map['client']
    clear_client(client)

def clear_client(cid):
    """Clean up connection if not already done."""
    pybullet.disconnect(physicsClientId=client)

def run_mc_trace(sim_map, state = None, fps = 60,
                 time_scale = 1.0):
    client = sim_map['client']
    world = sim_map['world']
    state = step_trace(client, world, 1./fps,
                       fps = fps, state = state,
                       time_scale = time_scale,
                       ret_col = False)
    return state[-1]

def run_full_trace(sim_map,
                   T = 1.0,
                   fps = 60,
                   time_scale = 1.0,
                   debug = False):
    """ Runs pybullet on on given scene
    Frames per second are fixed at 60
    If no objects are given, then the top keys of
    `serialized_scene` are used.

    :param data: Dict representing scene
    :param objs,optional: Order of objects to report.
    :param T: The duration in seconds to simulate
    :param fps: The rate at which to report state
    :param time_scale: The time scale factor
    :param debug: Whether to run with debug visualization
    """
    client = sim_map['client']
    world = sim_map['world']
    return step_trace(client, world, T,
                      fps = fps, time_scale = time_scale, debug = debug)

#######################################################################
# Helpers
#######################################################################

def _step_simulation(client, dt, time_step, update_fn):
    current = 0.0
    while current < dt:
        pybullet.stepSimulation(physicsClientId = client)
        update_fn()
        current += 1.0/time_step


def _ncr(n, r):
    r = min(r, n-r)
    numer = reduce(op.mul, range(n, n-r, -1), 1)
    denom = reduce(op.mul, range(1, r+1), 1)
    return numer // denom

# TODO : this is a duplicate of `Sim.update_obj`...
def _update_obj(obj_id, params, cid):
    if 'physics' in params:
        params = params['physics']
    params = clean_params(params)
    pybullet.changeDynamics(obj_id, -1,
                            **params,
                            physicsClientId = cid)
