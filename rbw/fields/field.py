from abc import ABC, abstractmethod


class Field(ABC):

    @abstractmethod
    def apply_field(self, graph):
        """
        Goes through the graph and returns edges to add
        """
        pass

    @abstractmethod
    def serialize(self):
        pass

    def __repr__(self):
        return self.serialize()

    def __str__(self):
        return str(self.serialize())
       

