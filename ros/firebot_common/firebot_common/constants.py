
import numpy as np

MAP_SIZE = 0.995  # 2.42

# Search
N_SEARCH_CELLS: int = 30  # 60
SEARCH_CELL_SIZE: float = MAP_SIZE / N_SEARCH_CELLS

# Distance
DIST_SUBSAMP: int = 1
N_DIST_CELLS: int = N_SEARCH_CELLS * DIST_SUBSAMP
DIST_CELL_SIZE: float = SEARCH_CELL_SIZE / DIST_SUBSAMP

# Robot
BODY_RADIUS = 0.18 / 2

def pos_to_search_cell(x: float, y: float):
    return int(y / SEARCH_CELL_SIZE), int(x / SEARCH_CELL_SIZE)

def pos_to_dist_cell(x: float, y: float):
    return int(y / DIST_CELL_SIZE), int(x / DIST_CELL_SIZE)

def search_cell_to_pos(r: int, c: int):
    return np.array([c * SEARCH_CELL_SIZE, r * SEARCH_CELL_SIZE])

def search_to_dist_cell(r: int, c: int):
    return r * DIST_SUBSAMP, c * DIST_SUBSAMP

def is_pos_outside(x: float, y: float):
    r, c = pos_to_search_cell(x, y)
    return not (r >= 0 and r < N_SEARCH_CELLS and c >= 0 and c < N_SEARCH_CELLS)
