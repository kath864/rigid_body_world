from . import Field
import numpy as np

class BeamField(Field):

    """
    A field that is directly applied to an object
    """

    def __init__(self,target, force = None, torque = None):
        self.target = target
        self.force = force
        self.torque = torque

    @property
    def force(self):
        return self._force

    @force.setter
    def force(self, v):
        if v is None:
            v = [0, 0, 0]
        else:
            if len(v) != 3:
                raise ValueError('force must be xyz')

        self._force = v

    @property
    def torque(self):
        return self._torque

    @torque.setter
    def torque(self, v):
        if v is None:
            v = [0, 0, 0]
        else:
            if len(v) != 3:
                raise ValueError('torque must be xyz')

        self._torque = v

    def apply_field(self, graph):
        force = np.array(self.force).tolist()
        torque = np.array(self.torque).tolist()
        return [(self.target, {'force' : force, 'torque':torque})]

    def serialize(self):
        return dict(
            target = self.target,
            force = np.array(self.force).tolist(),
            torque = np.array(self.torque).tolist()
            )
