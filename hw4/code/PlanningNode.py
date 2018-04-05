
class PlanningNode(object):

    def __init__(self, nid):
        self.id = nid
        self.closed = False
        self.fcost = float("inf")
        self.gcost = float("inf")
        self.parent = None
        self.action = None

class OpenList(object):
    
    def __init__(self):
        self.list = dict()

    def add(self, node):
        self.list[node.id] = node

    def lowest(self):

         min_node = None

         for node in self.list.values():
             if not node.closed and (min_node is None or node.fcost < min_node.fcost):
                 min_node = node

         return min_node

    def contains(self, nid):
        return (nid in self.list)
