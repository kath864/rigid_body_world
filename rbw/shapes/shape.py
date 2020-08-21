import numpy as np
from copy import deepcopy
from abc import ABC, abstractmethod
from . import KinematicObject

def change_prop(obj, prop, val):
    """ Returns a new object with updated physical property
    """
    if prop is None:
        return obj
    if not prop in obj.physics:
        raise ValueError('No prop {} in object'.format(prop))
    new_obj = deepcopy(obj)
    new_physics = new_obj.physics
    new_physics[prop] = val
    new_obj.physics = new_physics
    return new_obj

#TODO : doc me!
class Shape(KinematicObject):

    def __init__(self, appearance, dims, physics):
        super().__init__()
        self.appearance = appearance
        self.dimensions = dims
        self.physics = physics

    # ---------------- Abstract Methods -----------------#

    @property
    @abstractmethod
    def shape(self):
        """ The name of the shape
        :returns: A shape name
        :rtype: str
        """
        pass

    @property
    @abstractmethod
    def volume(self):
        """ Should be implemented by children.
        :returns: volume in m^3
        :rtype: float
        """
        pass

    @property
    @abstractmethod
    def dimensions(self):
        """ The xyz extremas of a given object type.
        """
        pass


    # ---------------- Properties -----------------#

    @property
    def physics(self):
        """ Returns a `dict` containing physical properties
        :returns: physical properties
        :rtype: dict
        """
        return self._physics

    @property
    def mass(self):
        """ Returns the mass in units
        :returns: volume * density
        :rtype: float
        """
        return self.physics['mass']

    @property
    def appearance(self):
        """ Returns the appearance descriptor"""
        return self._appearance

    @property
    def density(self):
        """ Returns the density in grams/m^3"""
        return self.physics['density']


    @property
    def friction(self):
        """ Returns the friction coefficient used by Bullet3D"""
        return self.physics['friction']

    # ----------------   setters   -----------------#

    @appearance.setter
    def appearance(self, value):
        """ set the appearance of the shape
        :param value: name of the texture
        :type value: str
        """
        self._appearance = value

    @physics.setter
    def physics(self, value):
        # TODO: add assertion for specific features
        value['mass'] = self.volume * value['density']
        self._physics = value

    # ----------------   Methods   -----------------#
    def serialize(self):
        """ Returns a json friendly dictionary representation
        :rtype: dict
        """
        d = super().serialize()
        d['appearance'] = self.appearance
        d['shape'] = self.shape
        d['dims'] = np.array(self.dimensions).tolist()
        d['volume'] = self.volume
        d.update(self.physics)
        return d
