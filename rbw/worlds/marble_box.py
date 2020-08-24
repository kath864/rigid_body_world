import networkx as nx
import numpy as np
from rbw.shapes import Block
from rbw.worlds import World

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

def make_table(table_dims, table_phys):
    g = nx.DiGraph()
    # for legacy support
    # pad dimensions with z
    if len(table_dims) == 2:
        table_dims = [*table_dims, 0.1]
    table = Block('table', table_dims, table_phys)
    # We want the table surface to be the xy plane @ z = 0
    table.position = [0, 0, table_dims[2]*-0.5]
    g.add_node('table_top', **table.serialize())

    # stair walls
    delta = 0.1
    exs_lr = np.array([delta, table_dims[1], 0.1])
    exs_fb = np.array([table_dims[0], delta, 0.1])

    pos_left =  table_dims * np.array([-0.5, 0, 0]) - np.array([delta, 0, 0])
    wall_left = Block('table', exs_lr, table_phys)
    wall_left.position = pos_left
    g.add_node('table_wall_left', **wall_left.serialize())

    pos_right =  table_dims * np.array([0.5, 0, 0]) + np.array([delta, 0, 0])
    wall_right = Block('table', exs_lr, table_phys)
    wall_right.position = pos_right
    g.add_node('table_wall_right', **wall_right.serialize())

    pos_front =  table_dims * np.array([0.0, 0.5, 0])
    wall_front = Block('table', exs_fb, table_phys)
    wall_front.position = pos_front
    g.add_node('table_wall_front', **wall_front.serialize())

    pos_back =  table_dims * np.array([0.0, -0.5, 0])
    wall_back = Block('table', exs_fb, table_phys)
    wall_back.position = pos_back
    g.add_node('table_wall_back', **wall_back.serialize())

    return g


class MarbleWorld(World):

    """ Describes a table with marbles

    Attributes
    ----------

    table : rbw.shapes.Block
       The geometry and physical parameters of the table


    """

    def __init__(self, table_dims,
                 table_phys = default_phys):
        g = make_table(table_dims, table_phys)
        self.graph = g

    @property
    def graph(self):
        return self._graph

    @graph.setter
    def graph(self, g):
        self._graph = g

    def add_object(self, name, obj, x, y):
        """ Places objects on the table

        Objects will be placed directly on the table's surface.

        Parameters
        ----------
        name : str
            The key to reference the object

        obj : rbw.shapes.Shape
            Object to add to table

        x : `float`
        The x location of the object.

        y : `float`
        The y location of the object.

        """
        z = obj.dimensions[-1] * 0.5
        obj.position = [x,y,z]
        self.graph.add_node(name, **obj.serialize())
