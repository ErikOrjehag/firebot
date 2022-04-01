from firebot_common.constants import N_SEARCH_CELLS, pos_to_search_cell, BODY_RADIUS, SEARCH_CELL_SIZE, search_to_dist_cell, search_cell_to_pos
import numpy as np
from queue import PriorityQueue
from math import sqrt


def flow(dijkstras, x, y):
    r, c = pos_to_search_cell(x, y)
    s = 1
    #print(x, y, r, c, dijkstras[r,c])
    patch = dijkstras[r-s:r+s+1,c-s:c+s+1].copy()
    patch -= dijkstras[r,c]
    # print("\n".join(" | ".join(f"{p:.3f}" for p in pp) for pp in patch))
    # print("-"*10)
    patch[1, 1] = 1e12
    dir = np.unravel_index(np.argmin(patch, axis=None), patch.shape) - np.array((1, 1))
    dir = search_cell_to_pos(*dir)
    dir /= np.linalg.norm(dir)
    return dir

def neighbours(r, c):
    for rr in range(r-1, r+2):
        for cc in range(c-1, c+2):
            if rr >= 0 and rr < N_SEARCH_CELLS and cc >= 0 and cc < N_SEARCH_CELLS and (rr != r or cc != c):
                yield rr, cc

def dijkstras_search(distmap, x: float, y: float):
    cost = np.ones((N_SEARCH_CELLS, N_SEARCH_CELLS)) * 1e12
    visited = np.zeros((N_SEARCH_CELLS, N_SEARCH_CELLS)).astype(np.bool)
    
    pq = PriorityQueue()
    
    r, c = pos_to_search_cell(x, y)

    if r < 0 or r >= N_SEARCH_CELLS or c < 0 or c >= N_SEARCH_CELLS:
        return cost

    cost[r, c] = 0.0
    pq.put((cost[r, c], (r, c)))

    while not pq.empty():
        _, (r, c) = pq.get()
        visited[r, c] = True
        for nr, nc in neighbours(r, c):
            if distmap[search_to_dist_cell(nr, nc)] > SEARCH_CELL_SIZE / 2:
                if not visited[nr, nc]:
                    d = SEARCH_CELL_SIZE * sqrt((nr-r)**2 + (nc-c)**2)
                    new_cost = cost[r, c] + d
                    if new_cost < cost[nr, nc]:
                        pq.put((new_cost, (nr, nc)))
                        cost[nr, nc] = new_cost
    return cost
