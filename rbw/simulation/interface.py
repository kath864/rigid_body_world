""" A collection of functions interfacing with pybullet simulations

Implemented as functions rather than classes to facilitate cross-language support.
"""
from rbw import np
from . import pybullet
import networkx as nx
from . import Sim

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

def init_sim_serialized(sim, str_data, cid):
    pass

def init_sim(sim, graph, cid):
    """ Loads a world into a client and returns an sim map

    Parameters
    ----------
    sim : Sim

    graph : nx.DiGraph

    cid : int

    Returns
    -------
    dict
        object ids
    """
    pass

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


#TODO: doc me!
def apply_state(sim, ids, state):
    w_rev = state.reverse()

    for name, node in w_rev.nodes(data = True):
        if not 'shape' in nodes:
            continue
        # load newtonian objects
        obj_id = ids[shape]
        sim.update_obj(obj_id, shape)
        self.apply_force_torque(obj_id, w_rev.adj[shape])

def batch_step(sim, ids, g, dur,
               time_step = 240,
               time_scale = 1.0,
               prev_state = None,
               debug = False):
    """Obtains sim state from simulation.

    Currently returns the position of each rigid body.
    The total duration is equal to `frames / fps`

    Arguments:
        dur (float): The amount of time in seconds to simulate
        objects ([str]): List of strings of objects to report
        time_step (int, optional): Number of physics steps per second
        fps (int, optional): Number of frames to report per second.
    """
    sim.setPhysicsEngineParameter(
        enableFileCaching = 0,
        fixedTimeStep = 1/time_step,
    )

    n_steps = int(dur * time_step)

    n_objs = len(ids)

    (prev_state is None) or apply_state(sim, ids, prev_state)

    if debug:
        # add one step to resolve any initial forces
        sim.stepSimulation()
        sim.setRealTimeSimulation(1)
        while True:
            keys = sim.getKeyboardEvents()
            print(keys)
            time.sleep(0.01)
        return

    states = np.empty(n_steps, g.__class__)
    for step in range(n_steps):
        states[step] = _step_simulation(sim, g, ids)

    return states


def clear_sim(sim_map):
    """Clean up connection if not already done."""
    # print('disconnecting from {0:d}'.format(client))
    client = sim_map['client']
    clear_client(client)

def clear_client(cid):
    """Clean up connection if not already done."""
    pybullet.disconnect(physicsClientId=cid)

def run_mc_trace(sim_map, state = None, fps = 60,
                 time_scale = 1.0):
    client = sim_map['client']
    world = sim_map['world']
    state = step_trace(client, world, 1./fps,
                       fps = fps, state = state,
                       time_scale = time_scale,
                       ret_col = False)
    return state[-1]

def run_full_trace(client, graph,
                   T = 1.0,
                   fps = 60,
                   time_scale = 1.0,
                   time_step = 240,
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
    sim = Sim(client)
    ids = sim.load_graph(graph)
    return batch_step(sim, ids, graph, T,
                      time_scale = time_scale,
                      time_step = time_step, debug = debug)

#######################################################################
# Helpers
#######################################################################

def _step_simulation(sim, g, ids):
    state = g.__class__()
    state.add_nodes_from(g)
    sim.stepSimulation()

    # node updates
    for name, obj_id in ids.items():
        node_state = sim.extract_state(obj_id)
        state.nodes[name].update(node_state)

    for c,(a,b) in enumerate(combinations(ids.keys(), 2)):
        contact_info = sim.getContactPoints(bodyA=ids[a],
                                            bodyB=ids[b])
        if len(contact_info) == 0:
            continue
        state.add_edge(a, b, distance = contact_info[8],
                       force = contact_info[9])
        contact_info = pybullet.getContactPoints(bodyA=ids[b],
                                                 bodyB=ids[a])
        state.add_edge(b, a, distance = contact_info[8],
                        force = contact_info[9])
    return state


def keypoints(states):
    current_kp = states[0]
    n = len(states)
    kps = {0 : (current_kp, [])}
    for t in range(1, n):
        g = states[t]
        d = difference(current_kp, g)
        if len(d.edges) > 0:
            kps[t] = (g, list(d.edges))
            current_kp = g
    return kps

# cribbed from https://stackoverflow.com/a/33494157
def difference(a, b):
    d = nx.create_empty_copy(b)
    if set(a) != set(b):
        raise nx.NetworkXError("Node sets of graphs is not equal")

    a_edges = set(a.edges())
    b_edges = set(b.edges())

    diff_edges = b_edges ^ a_edges

    if len(diff_edges) == 0:
        return d

    new_edges = b_edges - a_edges

    d.add_edges_from(new_edges, delta = 'added')
    d.add_edges_from(a_edges - b_edges, delta = 'removed')

    return d



