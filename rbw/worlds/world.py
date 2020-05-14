from abc import ABC, abstractmethod

class World(ABC):

    @property
    @abstractmethod
    def objects(self):
        """Returns a `dict` of added objects"""
        pass

    @abstractmethod
    def add_object(self):
        """Adds objects to the world"""
        pass

    @abstractmethod
    def serialize(self):
        """Returns a json-serialized context of the world"""
        d = {}
        d['objects'] = {k : o.serialize() for k,o in self.objects.items()}
        return d
