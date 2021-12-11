
import numpy as np
import math
import functools

@functools.total_ordering
class Node():

    def __init__(self, previous, posx, posy, angle, reversing):
        self.heap_index = None
        self.g_cost = 0
        self.h_cost = 0
        self.reversing = reversing
        self.pos = np.hstack((posx, posy))
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

    def __eq__(self, other):
        return self.f_cost == other.f_cost

    def __lt__(self, other):
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        else:
            return self.f_cost < other.f_cost

class Heap():

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

    def __len__(self):
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
                if self._items[right] < self._items[left]:
                    swap_index = right
            if item <= self._items[swap_index]:
                return
            self._swap(item, self._items[swap_index])

    def sort_up(self, item):
        while True:
            if item.heap_index == 0:
                return
            parent = (item.heap_index - 1) // 2
            if item >= self._items[parent]:
                return
            self._swap(item, self._items[parent])

    def _swap(self, itemA, itemB):
        self._items[itemA.heap_index], self._items[itemB.heap_index] = itemB, itemA
        itemA.heap_index, itemB.heap_index = itemB.heap_index, itemA.heap_index

def heuristic(node, to_x, to_y):
    return math.sqrt((to_x - node.x)**2 + (to_y - node.y)**2)

def get_children_of_node(node, walls, goal_x, goal_y, DRIVE_DIST, MAX_STEER):
    wheel_base = 0.15 # TODO: From Robot()

    children = []
    for drive_dist in [DRIVE_DIST, -DRIVE_DIST]:
        for steer in [-MAX_STEER, -MAX_STEER/2.0, 0.0, MAX_STEER/2.0, MAX_STEER]:
            turn_angle = (drive_dist / wheel_base) * math.tan(steer)
            
            if abs(turn_angle) < 1e-4:
                new_x = node.x + drive_dist * math.cos(node.angle) # TODO: sin/cos?
                new_y = node.y + drive_dist * math.sin(node.angle)
            else:
                R = drive_dist / turn_angle
                cx = node.x + math.sin(node.angle) * R
                cy = node.y - math.cos(node.angle) * R
                new_x = cx - math.sin(node.angle + turn_angle) * R
                new_y = cy + math.cos(node.angle + turn_angle) * R
            
            new_angle = node.angle + turn_angle
            if new_angle >= math.pi * 2.0:
                new_angle -= math.pi * 2.0
            elif new_angle < 0:
                new_angle += math.pi * 2.0
            
            # TODO is cell within map?

            is_reversing = (drive_dist < 0)
            child = Node(previous=node, posx=new_x, posy=new_y, angle=new_angle, reversing=is_reversing)
            child.h_cost = heuristic(child, goal_x, goal_y)
            d_cost = heuristic(child.previous, child.x, child.y)
            if child.reversing:
                d_cost *= 10.0
            child.g_cost = child.previous.g_cost + d_cost # TODO: https://github.com/Habrador/Self-driving-vehicle/blob/a38920c76a10727585309e14464857fc4695824c/Self-driving%20vehicle%20Unity/Assets/Scripts/Pathfinding/Hybrid%20A%20star/HybridAStar.cs#L616
            children.append(child)
    return children


def hybrid_astar_search(walls, start_x, start_y, start_angle, goal_x, goal_y, goal_angle = None):
    MAP_SIZE = 2.42
    N_CELLS = 20
    CELL_SIZE = MAP_SIZE / N_CELLS
    ANGLE_RESOLUTION = math.radians(20.0)
    DRIVE_DIST = math.sqrt((CELL_SIZE ** 2) * 2) + 0.01
    MAX_STEER = math.radians(30)
    POS_ACCURACY = CELL_SIZE
    ANGLE_ACCURACY = math.radians(30)

    # For debugging
    n_pruned_nodes = 0
    n_max_heap_size = 0
    all_expanded_nodes = []

    open_nodes = Heap()

    # int below is the rounded heading used to enter a cell
    closed_cells = [[set() for _y in range(N_CELLS)] for _x in range(N_CELLS)] # { heading(int), ... }
    # the node in the cell with the lowest g-cost at a certain heading
    lowest_cost_nodes = [[dict() for _y in range(N_CELLS)] for _x in range(N_CELLS)] # { heading(int): Node }

    start_cell = (int(start_x // CELL_SIZE), int(start_y // CELL_SIZE))
    goal_cell = (int(goal_x // CELL_SIZE), int(goal_y // CELL_SIZE))

    node = Node(previous=None, posx=start_x, posy=start_y, angle=start_angle, reversing=False)
    node.g_cost = 0.0
    node.h_cost = heuristic(node, goal_x, goal_y)

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
        if len(open_nodes) == 0:
            print('No more open nodes to explore!')
            resign = True
            break
        n_max_heap_size = max(n_max_heap_size, len(open_nodes))
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
        children = get_children_of_node(next_node, walls, goal_x, goal_y, DRIVE_DIST, MAX_STEER)
        for child in children:
            child_cell_pos = (int(child.x // CELL_SIZE), int(child.y // CELL_SIZE))
            if child_cell_pos[0] < 0 or child_cell_pos[1] < 0:
                n_pruned_nodes += 1
                continue
            if child_cell_pos[0] >= N_CELLS or child_cell_pos[1] >= N_CELLS:
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
                if cost_so_far < nodes_with_lowest_costs[child_cell_heading].g_cost:
                    existing_node = nodes_with_lowest_costs[child_cell_heading]
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

    print(f'{found=} {resign=} {final_node=}')
    return final_node


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
    hybrid_astar_search(mapp.walls, 0.2, 0.2, 0.0, 1.0, 1.0, 0.5)

def main():
    #test_heap()
    test_search()

if __name__ == '__main__':
    main()