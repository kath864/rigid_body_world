from rbw import np
from rbw.shapes import Block
from rbw.worlds import World

from collections import OrderedDict

default_phys = {'lateralFriction': 0.2,
                'density': 0.0}

table_dims = [1.8,1.8,0.1]

def default_box():
    scene = RampWorld(table_dims, ramp_dims,
                      ramp_angle = 30 * (np.pi/180.))
    return scene

class RampWorld(World):

    """
    Describes a ramp scene
    """

    def __init__(self, table_dims, ramp_dims, objects = None,
                 ramp_angle = 0.0, table_phys = default_phys,
                 ramp_phys = default_phys):
        # for legacy support
        # pad dimensions with z
        if len(table_dims) == 2:
            table_dims = [*table_dims, 0.1]
            ramp_dims = [*ramp_dims, 0.1]
        table = Block('table', table_dims, table_phys)
        # We want the table surface to be the xy plane @ z = 0
        table.position = pct_to_coord(table_dims[0]*0.5 , 0, -0.05)
        self.table = table
        ramp = Block('ramp', ramp_dims, ramp_phys,
                                angle = (0, ramp_angle, 0))
        ramp.position = pct_to_coord(ramp_dims[0]*(-0.5), ramp_angle, -0.05)
        # raised slightly to prevent snagging
        ramp.position += np.array([0, 0, 1E-2])
        self.ramp = ramp
        self.ramp_angle = ramp_angle
        self.objects = objects
        self.initial_pos = None

    @property
    def objects(self):
        if self._objects is None:
            return OrderedDict()
        else:
            return self._objects

    @objects.setter
    def objects(self, v):
        self._objects = v

    @property
    def initial_pos(self):
        if self._initial_pos is None:
            return OrderedDict()
        else:
            return self._initial_pos

    @initial_pos.setter
    def initial_pos(self, v):
        self._initial_pos = v

    def add_object(self, name, obj, place):
        z = obj.dimensions[-1]
        # on table
        if place < 1:
            mag = place * self.table.dimensions[0]
            angle = 0
        # on ramp
        elif place < 2 and place > 1:
            mag = (1 - place) * self.ramp.dimensions[0]
            angle = self.ramp_angle
        else:
            raise ValueError('Place not found')

        pos = pct_to_coord(mag, angle, z/2)
        obj.position = pos
        obj.orientation = (0, angle, 0)
        objects = self.objects
        objects[name] = obj
        self.objects = objects
        initial_pos = self.initial_pos
        initial_pos[name] = place
        self.initial_pos = initial_pos

    def serialize(self):
        # add 'objects'
        d = super().serialize()
        d['ramp'] = self.ramp.serialize()
        d['table'] = self.table.serialize()
        d['initial_pos'] = self.initial_pos
        return d

from rbw import np
from rbw.shapes import Block
from rbw.worlds import World

from collections import OrderedDict

def pct_to_coord(mag, angle, z):
    x_offset = (np.cos(angle) * mag) + (np.sin(angle) * z)
    z_offset = (np.sin(angle) * abs(mag)) + (np.cos(angle) * z)
    return np.array([x_offset, 0, z_offset])

default_phys = {'lateralFriction': 0.2,
                'density': 0.0}

table_dims = [3.5,1.8,0.1]
ramp_dims = [3.5,1.8,0.1]

def default_ramp():
    scene = RampWorld(table_dims, ramp_dims,
                      ramp_angle = 30 * (np.pi/180.))
    return scene

class RampWorld(World):

    """
    Describes a ramp scene
    """

    def __init__(self, table_dims, ramp_dims, objects = None,
                 ramp_angle = 0.0, table_phys = default_phys,
                 ramp_phys = default_phys):
        # for legacy support
        # pad dimensions with z
        if len(table_dims) == 2:
            table_dims = [*table_dims, 0.1]
            ramp_dims = [*ramp_dims, 0.1]
        table = Block('table', table_dims, table_phys)
        # We want the table surface to be the xy plane @ z = 0
        table.position = pct_to_coord(table_dims[0]*0.5 , 0, -0.05)
        self.table = table
        ramp = Block('ramp', ramp_dims, ramp_phys,
                                angle = (0, ramp_angle, 0))
        ramp.position = pct_to_coord(ramp_dims[0]*(-0.5), ramp_angle, -0.05)
        # raised slightly to prevent snagging
        ramp.position += np.array([0, 0, 1E-2])
        self.ramp = ramp
        self.ramp_angle = ramp_angle
        self.objects = objects
        self.initial_pos = None

    @property
    def objects(self):
        if self._objects is None:
            return OrderedDict()
        else:
            return self._objects

    @objects.setter
    def objects(self, v):
        self._objects = v

    @property
    def initial_pos(self):
        if self._initial_pos is None:
            return OrderedDict()
        else:
            return self._initial_pos

    @initial_pos.setter
    def initial_pos(self, v):
        self._initial_pos = v

    def add_object(self, name, obj, place):
        z = obj.dimensions[-1]
        # on table
        if place < 1:
            mag = place * self.table.dimensions[0]
            angle = 0
        # on ramp
        elif place < 2 and place > 1:
            mag = (1 - place) * self.ramp.dimensions[0]
            angle = self.ramp_angle
        else:
            raise ValueError('Place not found')

        pos = pct_to_coord(mag, angle, z/2)
        obj.position = pos
        obj.orientation = (0, angle, 0)
        objects = self.objects
        objects[name] = obj
        self.objects = objects
        initial_pos = self.initial_pos
        initial_pos[name] = place
        self.initial_pos = initial_pos

    def serialize(self):
        # add 'objects'
        d = super().serialize()
        d['ramp'] = self.ramp.serialize()
        d['table'] = self.table.serialize()
        d['initial_pos'] = self.initial_pos
        return d
