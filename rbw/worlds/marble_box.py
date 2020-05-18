from rbw import np
from rbw.shapes import Block
from rbw.worlds import World

from collections import OrderedDict

default_phys = {'lateralFriction': 0.2,
                'density': 0.0}

table_dims = [1.8,1.8,0.1]

def default_box():
    """ Returns a default empty marble world.

    The default marble world with a table of `table_dims`
    and physics of `default_phys`.
    """
    tbl = MarbleWorld(table_dims)
    return tbl

class MarbleWorld(World):

    """ Describes a table with marbles

    Attributes
    ----------

    table : rbw.shapes.Block
       The geometry and physical parameters of the table


    """

    def __init__(self, table_dims,
                 table_phys = default_phys,
                 objects = None):
        # for legacy support
        # pad dimensions with z
        if len(table_dims) == 2:
            table_dims = [*table_dims, 0.1]
        table = Block('table', table_dims, table_phys)
        # We want the table surface to be the xy plane @ z = 0
        table.position = [0, 0, table_dims[2]*-0.5]
        self.table = table
        self.objects = objects
        self.init_vel = {}

    @property
    def objects(self):
        return self._objects

    @objects.setter
    def objects(self, v):
        if v is None:
            v = OrderedDict()
        self._objects = v

    def add_object(self, name, obj, x, y,
                   lin_vel = None):
        """ Places objects on the table

        Objects will be placed directly on the table's surface.

        Parameters
        ----------
        name : str
        The key to reference the object

        obj : ``rbw.shapes.Shape``
        Object to add to table

        x : `float`
        The x location of the object.
        y : `float`
        The y location of the object.
        """
        z = obj.dimensions[-1] * 0.5
        obj.position = [x,y,z]

        objects = self.objects
        objects[name] = obj
        self.objects = objects

        if not (lin_vel is None):
            self.init_vel[name] = lin_vel


    def serialize(self):
        """ Serializes an instance of `MarbleWorld`

        Returns
        -------
        dict

        The serialized instace will have the following format

        ``
        'objects' : ...,
        'table' : ...
        ``
        """
        # add 'objects'
        d = super().serialize()
        d['table'] = self.table.serialize()
        d['init_lin_vel'] = self.init_vel
        return d
