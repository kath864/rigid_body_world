from rbw import np
from . import Sim

# TODO document gravity

class MarbleSim(Sim):

    """
    Handles physics for `rbw.shapes.MarbleWorld`


    Objects with an initial velocity are configured.

    Defines a method `make_table` to describe a table top with bounderies along each edge.


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
        self.make_table(w['table'])
        init_force = w['init_force']
        d = {}
        for obj,data in w['objects'].items():
            d[obj] = self.make_obj(data)
            if obj in w['init_force']:
                f = init_force[obj]
                self.applyExternalForce(d[obj], -1, f, [0,0,0],
                                        self.LINK_FRAME)
        self._world = d


    #-------------------------------------------------------------------------#
    # Methods

    def make_table(self, params):
        # Table top
        base_id = self.make_obj(params)

        # table walls
        shape = self.GEOM_BOX
        exs = np.array(params['dims']) / 2.0

        rot = self.getQuaternionFromEuler((np.pi/2, 0, 0))
        wall_left = self.createCollisionShape(shape, halfExtents = exs)
        pos_left =  [0, exs[1] + exs[2], 0]
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_left,
                                      basePosition = pos_left,
                                      baseOrientation = rot)
        self.changeDynamics(obj_id, -1)
        pos_right =  [0, -1 * (exs[1] + exs[2]), 0]
        wall_right = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_right,
                                      basePosition = pos_right,
                                      baseOrientation = rot)
        self.changeDynamics(obj_id, -1)
        pos_front =  [(exs[1] + exs[2]), 0, 0]
        rot = self.getQuaternionFromEuler((0, np.pi/2, 0))
        wall_front = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_front,
                                      basePosition = pos_front,
                                      baseOrientation = rot,)
        self.changeDynamics(obj_id, -1)
        pos_back =  [-1*(exs[1] + exs[2]), 0, 0]
        wall_back = self.createCollisionShape(shape, halfExtents = exs)
        obj_id = self.createMultiBody(baseCollisionShapeIndex = wall_back,
                                      basePosition = pos_back,
                                      baseOrientation = rot,)
        self.changeDynamics(obj_id, -1)
        return base_id
