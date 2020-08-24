""" A collection of functions interfacing with pybullet simulations

Implemented as functions rather than classes to facilitate cross-language support.
"""
import time
from rbw import np
from . import pybullet
import networkx as nx
from . import Sim

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

def update_world(sim, ids, new_world):
    """ Updates the world's physical parameters

    :param client: Pybullet client id
    :param object_map: The object name -> object id `dict`
    :new_world: A serialized scene containing new physical properties
    """
    for name, node in new_world.nodes(data = True):
        sim.update_object(ids[name], node)


#TODO: doc me!
def apply_state(sim, ids, state):
    w_rev = state.reverse()
    for name, node in w_rev.nodes(data = True):
        if not 'shape' in nodes:
            continue
        # load newtonian objects
        obj_id = ids[name]
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

    if not (prev_state is None):
        apply_state(sim, ids, prev_state)

    if debug:
        # add one step to resolve any initial forces
        sim.stepSimulation()
        sim.setRealTimeSimulation(1)
        while True:
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
# state reporting
#######################################################################

def keypoints(states):
    """ The time points and graphs at moments where new contacts form or dissolve

    Parameters
    ----------

    states : np.array
        an array containing nx.digraph state representations

    Returns
    -------

    dict
        Keys represent contact change points. Values are a tuple where
        lhs is the graph at time t, and the rhs is itself a tuple of
        (new edges, removed edges).

    """
    current_kp = states[0]
    n = len(states)
    kps = {0 : (current_kp, [])}
    for t in range(1, n):
        g = states[t]
        d = difference(current_kp, g)
        if len(d) > 0:
            kps[t] = (g, d)
            current_kp = g
    return kps

def parse_state(states, names, hz):
    """ Collects the states of objects in space-time

    Parameters
    ----------

    states : np.array
        an array containing nx.DiGraph state representations

    names : iterable(str)
        Nodes to collect

    hz : int
        how many collections to report per state step

    Returns
    -------

    something
    """
    n = len(states)
    ts = np.arange(n, step = hz)

    r = {}
    for o in names:
        r[o] = _empty_state(len(ts))

    for i,t in enumerate(ts):
        data = states[t].nodes(data = True)
        for o in names:
            for k,v in data[o].items():
                r[o][k][i] = v

    return r

#######################################################################
# helpers
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
        contact_info = contact_info[0]
        state.add_edge(a, b, distance = contact_info[8],
                       force = contact_info[9])
        contact_info = pybullet.getContactPoints(bodyA=ids[b],
                                                 bodyB=ids[a])
        contact_info = contact_info[0]
        state.add_edge(b, a, distance = contact_info[8],
                        force = contact_info[9])
    return state


# cribbed from https://stackoverflow.com/a/33494157
def difference(a, b):
    d = nx.create_empty_copy(b)
    if set(a) != set(b):
        raise nx.NetworkXError("Node sets of graphs is not equal")

    a_edges = set(a.edges())
    b_edges = set(b.edges())

    diff_edges = b_edges ^ a_edges

    if len(diff_edges) == 0:
        return tuple()

    new_edges = b_edges - a_edges
    removed = a_edges - b_edges

    return (new_edges, removed)

def _empty_state(t):
    return {
        'position' : np.empty((t, 3)),
        'orientation' : np.empty((t, 4)),
        'linear_vel' : np.empty((t, 3)),
        'angular_vel' : np.empty((t, 3)),
    }
