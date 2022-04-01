
import numpy as np
from math import sqrt, cos, sin, pi, radians
import functools

import firebot_common.wall_collision
from firebot_common.constants import SEARCH_CELL_SIZE, pos_to_search_cell, pos_to_dist_cell, is_pos_outside

class Node:
    def __init__(self, previous, posx, posy, angle, reversing):
        self.heap_index = -1
        self.g_cost = 0.0
        self.h_cost = 0.0
        self.reversing = reversing
        self.pos = np.array([posx, posy])
        self.angle = angle
        self.previous = previous

    def __repr__(self):
        return f'{self.heap_index:>03d} -> g_cost={self.g_cost:2f}, h_cost={self.h_cost:2f}, f_cost={self.f_cost:2f}'

    def give_my_data_to(self, other):
        other.g_cost = self.g_cost
        other.h_cost = self.h_cost
        other.reversing = self.reversing
        other.x = self.x
        other.y = self.y
        other.angle = self.angle
        other.previous = self.previous

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    @property
    def x(self):
        return self.pos[0]

    @property
    def y(self):
        return self.pos[1]

    @x.setter
    def x(self, x):
        self.pos[0] = x
    
    @y.setter
    def y(self, y):
        self.pos[1] = y

    def eq(self, other):
        return self.f_cost == other.f_cost

    def lt(self, other):
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        else:
            return self.f_cost < other.f_cost
    
    def lte(self, other):
        return self.eq(other) or self.lt(other)
    
    def gte(self, other):
        return not (self.lt(other))
    
    def gt(self, other):
        return not self.eq(other) and self.gte(other)


class Heap:

    def __init__(self):
        self._items = []

    def __repr__(self):
        return self._repr_recuse(0, 0)

    def _repr_recuse(self, index, depth):
        if index >= len(self._items):
            return None
        left = index * 2 + 1
        right = index * 2 + 2
        r = self._repr_recuse(right, depth+1)
        m = (' '*depth*10) + repr(self._items[index])
        l = self._repr_recuse(left, depth+1)
        return (r + '\n' if r else '') + m + ('\n' + l if l else '')

    def size(self):
        return len(self._items)

    def add(self, item):
        item.heap_index = len(self._items)
        self._items.append(item)
        self.sort_up(item)

    def pop_first(self):
        first = self._items[0]
        last = self._items.pop()
        if len(self._items) > 0:
            last.heap_index = 0
            self._items[0] = last
            self.sort_down(last)
        return first

    def clear(self):
        self._items.clear()

    def sort_down(self, item):
        while True:
            left = item.heap_index * 2 + 1
            right = item.heap_index * 2 + 2
            if left >= len(self._items):
                return
            swap_index = left
            if right < len(self._items):
                if self._items[right].lt(self._items[left]):
                    swap_index = right
            if item.lte(self._items[swap_index]):
                return
            self._swap(item, self._items[swap_index])

    def sort_up(self, item):
        while True:
            if item.heap_index == 0:
                return
            parent = (item.heap_index - 1) // 2
            if item.gte(self._items[parent]):
                return
            self._swap(item, self._items[parent])

    def _swap(self, itemA, itemB):
        self._items[itemA.heap_index], self._items[itemB.heap_index] = itemB, itemA
        itemA.heap_index, itemB.heap_index = itemB.heap_index, itemA.heap_index

def heuristic(node, dijkstras):
    return dijkstras[pos_to_search_cell(node.x, node.y)]

def get_children_of_node(distmap, node, walls, dijkstras, DRIVE_DIST, MAX_STEER):
    wheel_base = 0.15 # TODO: From Robot()

    max_dist = distmap.max()

    children = []
    for drive_dist in [DRIVE_DIST, -DRIVE_DIST]:
        for steer in [-MAX_STEER, -MAX_STEER/2.0, 0.0, MAX_STEER/2.0, MAX_STEER]:
            new_x = node.x + drive_dist * cos(node.angle)
            new_y = node.y + drive_dist * sin(node.angle)
            
            if is_pos_outside(new_x, new_y):
                continue

            new_angle = node.angle + steer
            if new_angle >= pi * 2.0:
                new_angle -= pi * 2.0
            elif new_angle < 0:
                new_angle += pi * 2.0

            is_reversing = (drive_dist < 0)
            child = Node(node, new_x, new_y, new_angle, is_reversing)
            child.h_cost = heuristic(child, dijkstras)
            dist = distmap[pos_to_dist_cell(child.x, child.y)]
            d_cost = SEARCH_CELL_SIZE * sqrt((child.previous.x - child.x)**2 + (child.previous.y - child.y)**2) # heuristic(child.previous, child.x, child.y) # + 10.0 * (1.0 - max(dist, 0.0001) / max_dist)
            if child.reversing:
                d_cost *= 20.0
            r_cost = 0.0
            if not child.previous.reversing and child.reversing:
                r_cost = 10.0
            t_cost = 0.5 * abs(steer)
            child.g_cost = child.previous.g_cost + d_cost  # + r_cost + t_cost # TODO: https://github.com/Habrador/Self-driving-vehicle/blob/a38920c76a10727585309e14464857fc4695824c/Self-driving%20vehicle%20Unity/Assets/Scripts/Pathfinding/Hybrid%20A%20star/HybridAStar.cs#L616
            children.append(child)
    return children

MAP_SIZE = 2.42
N_CELLS = 20
CELL_SIZE = MAP_SIZE / N_CELLS

def hybrid_astar_search(walls, distmap, dijkstras, start_x, start_y, start_angle, goal_x, goal_y, goal_angle = None):
    ANGLE_RESOLUTION = radians(15.0)
    DRIVE_DIST = sqrt((CELL_SIZE ** 2) * 2) + 0.01
    MAX_STEER = radians(40)
    POS_ACCURACY = CELL_SIZE
    ANGLE_ACCURACY = radians(30)

    body_radius = 0.18/2 # TODO: From Robot()

    # For debugging
    n_pruned_nodes = 0
    n_max_heap_size = 0
    all_expanded_nodes = []

    open_nodes = Heap()

    # int below is the rounded heading used to enter a cell : types.ListType(types.ListType(types.Set(int64, reflected=True)))
    closed_cells = [[set({0}) for _y in range(N_CELLS)] for _x in range(N_CELLS)] # { heading(int), ... }
    # the node in the cell with the lowest g-cost at a certain heading {0: Node(None, 0.0, 0.0, 0.0, False) for _ in range(0)}
    lowest_cost_nodes = [[{} for _y in range(N_CELLS)] for _x in range(N_CELLS)]#[[typed.Dict.empty(key_type=int64,value_type=Node) for _y in range(N_CELLS)] for _x in range(N_CELLS)] # { heading(int): Node }

    start_cell = (int(start_x // CELL_SIZE), int(start_y // CELL_SIZE))
    goal_cell = (int(goal_x // CELL_SIZE), int(goal_y // CELL_SIZE))

    node = Node(previous=None, posx=start_x, posy=start_y, angle=start_angle, reversing=False)
    node.g_cost = 0.0
    node.h_cost = heuristic(node, dijkstras)

    open_nodes.add(node)

    final_node = None
    found = False
    resign = False

    iterations = 0

    while not found and not resign:
        if iterations > 10000:
            print('Too many iterations!')
            break
        iterations += 1
        if open_nodes.size() == 0:
            print('No more open nodes to explore!')
            resign = True
            break
        n_max_heap_size = max(n_max_heap_size, open_nodes.size())
        next_node = open_nodes.pop_first()
        cell_pos = (int(next_node.x // CELL_SIZE), int(next_node.y // CELL_SIZE))
        cell_heading = int(next_node.angle // ANGLE_RESOLUTION)
        closed_in_this_cell = closed_cells[cell_pos[0]][cell_pos[1]]
        if cell_heading in closed_in_this_cell:
            iterations -= 1
            continue
        closed_in_this_cell.add(cell_heading)
        all_expanded_nodes.append(next_node)
        goal_dist_sqrd = (goal_x - next_node.x) ** 2 + (goal_y - next_node.y) ** 2
        heading_diff = abs(goal_angle - next_node.angle)
        if (goal_dist_sqrd < POS_ACCURACY*POS_ACCURACY or cell_pos == goal_cell) and heading_diff < ANGLE_ACCURACY:
            print('Found path!')
            found = True
            final_node = next_node
            break
        children = get_children_of_node(distmap, next_node, walls, dijkstras, DRIVE_DIST, MAX_STEER)
        for child in children:
            child_cell_pos = (int(child.x // CELL_SIZE), int(child.y // CELL_SIZE))
            if child_cell_pos[0] < 0 or child_cell_pos[1] < 0:
                n_pruned_nodes += 1
                continue
            if child_cell_pos[0] >= N_CELLS or child_cell_pos[1] >= N_CELLS:
                n_pruned_nodes += 1
                continue
            is_distmap_collision = distmap[int(child.pos[1] / firebot_common.wall_collision.CELL_SIZE), int(child.pos[0] / firebot_common.wall_collision.CELL_SIZE)] < body_radius*1.5
            #if firebot_common.wall_collision.is_wall_collision(child.pos, body_radius*1.5, walls):
            if is_distmap_collision:
                n_pruned_nodes += 1
                continue
            child_cell_heading = int(child.angle // ANGLE_RESOLUTION)
            closed_in_child_cell = closed_cells[child_cell_pos[0]][child_cell_pos[1]]
            if child_cell_heading in closed_in_child_cell:
                n_pruned_nodes += 1
                continue
            cost_so_far = child.g_cost
            nodes_with_lowest_costs = lowest_cost_nodes[child_cell_pos[0]][child_cell_pos[1]]
            if child_cell_heading in nodes_with_lowest_costs:
                existing_node: Node = nodes_with_lowest_costs[child_cell_heading]
                assert existing_node is not None, "fuu"
                #g_cost: float64 = existing_node.g_cost
                g_cost = 0.1
                if cost_so_far < g_cost:
                    child.give_my_data_to(existing_node)
                    open_nodes.sort_up(existing_node)
                else:
                    n_pruned_nodes += 1
                continue
            nodes_with_lowest_costs[child_cell_heading] = child
            if False: # TODO: Invalid position
                n_pruned_nodes += 1
                continue
            open_nodes.add(child)

    #print(f'found={found} resign={resign} final_node={final_node}')
    print("found", found, "resign", resign)

    if final_node is None:
        return None
    else:
        path = []
        node = final_node
        while node is not None:
            path.append([node.x, node.y, node.angle])
            node = node.previous
        path.reverse()
        path = np.array(path)
        return path

def smooth_path(path):
    x = path[:,:2]
    y = x.copy()
    alpha = 0.5
    beta = 0.1
    for _ in range(1000):
        y[1:-1] += alpha * (x[1:-1] - y[1:-1]) + beta * (y[2:] + y[:-2] - (2 * y[1:-1]))
    for p, yy in zip(path, y):
        p[0] = yy[0]
        p[1] = yy[1]

def test_heap():
    import random
    heap = Heap()
    for _ in range(5):
        node = Node(None, 0.0, 0.0, 0.0, False)
        node.g_cost = random.random()
        node.h_cost = random.random()
        heap.add(node)
        print(heap)
        print('-'*10)
    f = heap.pop_first()
    print(f'popped: {f}')
    print('-'*10)
    print(heap)
    print('-'*10)
    item2 = heap._items[2]
    item2.g_cost /= 4
    item2.h_cost /= 4
    print(item2)
    print('-'*10)
    heap.sort_up(item2)
    print(heap)
    print('-'*10)

def test_search():
    from mapp import Map
    mapp = Map()
    final_node = hybrid_astar_search(mapp.walls, 0.2, 0.2, 0.0, 1.0, 1.0, 0.5)
    smooth_path(final_node)

def main():
    #test_heap()
    test_search()

if __name__ == '__main__':
    main()