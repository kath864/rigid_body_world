import numpy as np
from abc import ABC, abstractmethod

#TODO doc me!
class KinematicObject(ABC):

    def __init__(self):
        self.position = None
        self.orientation = None
        self.linear_velocity = None
        self.angular_velocity = None

    # ---------------- Properties -----------------#

    @property
    def position(self):
        """ The XYZ coordinate of the object's COM"""
        return self._pos

    @property
    def orientation(self):
        """ The wxyz quaternion for object's rotation"""
        return self._orien

    @property
    def linear_velocity(self):
        return self._lin_vel

    @property
    def angular_velocity(self):
        return self._ang_vel

    # ----------------   setters   -----------------#

    @position.setter
    def position(self, value):
        if value is None:
            value = [0, 0, 0]
        v = np.array(value)
        if v.size != 3:
            raise ValueError('position must be xyz')
        self._pos = v

    @orientation.setter
    def orientation(self, value):
        if value is None:
            value = [0, 0, 0]
        v = np.array(value)
        if not (v.size == 3 or v.size == 4):
            msg = 'Orientation must be 3 euler angles or 4 wxyz quaternion'
            raise ValueError(msg)
        self._orien = v

    @linear_velocity.setter
    def linear_velocity(self, value):
        if value is None:
            value = [0, 0, 0]
        v = np.array(value)
        if v.size != 3:
            raise ValueError('linear vel must be xyz')
        self._lin_vel = v

    @angular_velocity.setter
    def angular_velocity(self, value):
        if value is None:
            value = [0, 0, 0]
        v = np.array(value)
        if v.size != 3:
            raise ValueError('angular vel must be xyz')
        self._ang_vel = v

    # ----------------   Methods   -----------------#
    def serialize(self):
        """ Returns a json friendly dictionary representation
        :rtype: dict
        """
        d = {'position' : self.position.tolist(),
             'orientation' : self.orientation.tolist(),
             'linear_velocity' : self.linear_velocity.tolist(),
             'angular_velocity' : self.angular_velocity.tolist()}
        return d


    def __repr__(self):
        return self.serialize()

    def __str__(self):
        return str(self.serialize())
