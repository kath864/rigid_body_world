from rbw import np
from . import Sim


class RampSim(Sim):

    """
    Handles physics for ramp worlds.

    Defines a method `make_table` to describe a table top with 3 edge boundaries (+x, +y, -y)
    """

    def __init__(self, scene_json, client):
        self.client = client
        self.world = scene_json

    #-------------------------------------------------------------------------#
    # Attributes

    @property
    def client(self):
        return self._client

    @client.setter
    def client(self, cid):
        if cid < 0:
            raise ValueError('Client is offline')
        self._client = cid

    @property
    def world(self):
        return self._world

    @world.setter
    def world(self, w):
        self.resetSimulation()
        self.setGravity(0, 0, -10)
        self.make_obj(w['ramp'])
        self.make_table(w['table'])
        d = {}
        for obj,data in w['objects'].items():
            d[obj] = self.make_obj(data)
        self._world = d


    #-------------------------------------------------------------------------#
    # Methods

    def make_table(self, params):
        # Table top
        base_id = self.make_obj(params)

        shape = self.GEOM_BOX
        exs = np.array(params['dims']) / 2.0
        # table walls
        rot = self.getQuaternionFromEuler((np.pi/2, 0, 0))
        wall_left = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_left,
                                          basePosition = [params['position'][0], exs[1], 0],
                                          baseOrientation = rot)
        self.changeDynamics(obj_id, -1)
        wall_right = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_right,
                                   basePosition = [params['position'][0], -exs[1], 0],
                                   baseOrientation = rot)
        self.changeDynamics(obj_id, -1)
        rot = self.getQuaternionFromEuler((0, np.pi/2, 0))
        wall_end = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_end,
                                   basePosition = [exs[0]*2+exs[2], 0, 0],
                                   baseOrientation = rot,)
        self.changeDynamics(obj_id, -1)
        return base_id
