from rbw import np
from . import pybullet
import inspect
import functools
from abc import ABC, abstractmethod
import networkx as nx


phys_keys = ['lateralFriction', 'mass', 'restitution',
             'rollingFriction', 'linearDamping']

def is_phys_key(kv):
    return (kv[0] in phys_keys)

def clean_params(phys_params):
    return dict(filter(is_phys_key, phys_params.items()))

def is_shape_node(node):
    return 'shape' in node

class Sim(ABC):

    def __init__(self, cid):
        self.client = cid

    # cribbed from:
    # https://github.com/bulletphysics/bullet3/blob/004dcc34041d1e5a5d92f747296b0986922ebb96/examples/pybullet/gym/pybullet_utils/bullet_client.py#L41-L48
    def __getattr__(self, name):
        """Inject the client id into Bullet functions."""
        attribute = getattr(pybullet, name)
        if inspect.isbuiltin(attribute):
            attribute = functools.partial(attribute,
                                          physicsClientId=self.client)
        return attribute


    def load_graph(self, w):
        self.resetSimulation()
        self.setGravity(0, 0, -10)

        w_rev = w.reverse()
        ids = {}
        for name, node in w_rev.nodes(data = True):
            if not 'shape' in node:
                continue
            # load newtonian objects
            obj = self.make_shape(node)

            self.apply_force_torque(obj, w_rev.adj[name])
            # record obj id
            ids[name] = obj

        return ids

    def make_shape(self, node):
        if node['shape'] == 'Block':
            mesh = self.GEOM_BOX
            dims = np.array(node['dims']) / 2.0
            col_id = self.createCollisionShape(mesh, halfExtents = dims)
        elif node['shape'] == 'Puck':
            mesh = self.GEOM_CYLINDER
            radius, _, height = np.array(node['dims'])
            col_id = self.createCollisionShape(mesh,
                                            radius = radius / 2.0,
                                            height = height)
        elif node['shape'] == 'Ball':
            mesh = self.GEOM_SPHERE
            z = node['dims'][0] * 0.5
            col_id = self.createCollisionShape(mesh, radius = z)
        else:
            raise ValueError("Unknown shape {}".format(node['shape']))

        obj_id = self.createMultiBody(baseCollisionShapeIndex = col_id)
        self.update_obj(obj_id, node)
        return obj_id

    def update_obj(self, obj_id, node):
        rot = node['orientation']
        if len(rot) == 3:
            rot = self.getQuaternionFromEuler(rot)

        self.resetBasePositionAndOrientation(obj_id,
                                             posObj = node['position'],
                                             ornObj = rot)
        self.resetBaseVelocity(obj_id,
                               linearVelocity = node['linear_velocity'],
                               angularVelocity = node['angular_velocity'])
        phys = clean_params(node)
        if len(phys) > 0:
            self.changeDynamics(obj_id, -1, **phys)

    def apply_force_torque(self, obj, edges):
        # apply forces and torques
        force = np.zeros(3)
        torque = np.zeros(3)
        for nbr, e in edges.items():
            force += e['force']
            torque += e['torque']

        f = self.applyExternalForce(obj, -1, force, [0,0,0],
                                    self.LINK_FRAME)
        self.applyExternalTorque(obj, -1, torque, self.LINK_FRAME)


    def extract_state(self, obj_id):
        pos, quat = self.getBasePositionAndOrientation(obj_id)
        l_vel, a_vel = self.getBaseVelocity(obj_id)
        return {'position' : pos, 'orientation' : quat,
                'linear_vel' : l_vel, 'angular_vel' : a_vel }

    # def serialize(self):
    #     return {'world' : self.world,
    #             'client' : self.client}
