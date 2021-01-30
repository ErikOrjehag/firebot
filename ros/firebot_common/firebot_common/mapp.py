
import firebot_common.data as data
import numpy as np

class Map():

    def __init__(self):
        verts = [np.array(v)/100. for v in data.vertices]
        walls = [np.concatenate((verts[w[0]], verts[w[1]])) for w in data.walls]
        self.verts = np.vstack(verts)
        self.walls = np.vstack(walls)