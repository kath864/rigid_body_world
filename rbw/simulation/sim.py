from rbw import np
from . import pybullet
import inspect
import functools
from abc import ABC, abstractmethod


phys_keys = ['lateralFriction', 'mass', 'restitution',
             'rollingFriction', 'linearDamping']

def clean_params(phys_params):
    return {k:v for k,v in phys_params.items() if k in phys_keys}

class Sim(ABC):


    @property
    @abstractmethod
    def client(self):
        """ The pybullet client key"""
        pass

    @property
    @abstractmethod
    def world(self):
        """ A `Dict` containing references to loaded objects"""
        pass

    # cribbed from:
    # https://github.com/bulletphysics/bullet3/blob/004dcc34041d1e5a5d92f747296b0986922ebb96/examples/pybullet/gym/pybullet_utils/bullet_client.py#L41-L48
    def __getattr__(self, name):
        """Inject the client id into Bullet functions."""
        attribute = getattr(pybullet, name)
        if inspect.isbuiltin(attribute):
            attribute = functools.partial(attribute, physicsClientId=self.client)
        return attribute

    def make_obj(self, params):
        if params['shape'] == 'Block':
            mesh = self.GEOM_BOX
            dims = np.array(params['dims']) / 2.0
            col_id = self.createCollisionShape(mesh, halfExtents = dims)
        elif params['shape'] == 'Puck':
            mesh = self.GEOM_CYLINDER
            radius, _, height = np.array(params['dims'])
            col_id = self.createCollisionShape(mesh,
                                            radius = radius / 2.0,
                                            height = height)
        else:
            mesh = self.GEOM_SPHERE
            z = params['dims'][0]
            col_id = self.createCollisionShape(mesh, radius = z)

        rot = self.getQuaternionFromEuler(params['orientation'])
        obj_id = self.createMultiBody(baseCollisionShapeIndex = col_id,
                                      basePosition = params['position'],
                                      baseOrientation = rot)
        self.update_obj(obj_id, params)
        return obj_id

    def update_obj(self, obj_id, params):
        if 'physics' in params:
            params = params['physics']
        params = clean_params(params)
        self.changeDynamics(obj_id, -1, **params)
