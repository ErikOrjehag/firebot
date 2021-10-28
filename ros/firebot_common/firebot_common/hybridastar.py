
import numpy as np
import heapq
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

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

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
        self._items[0] = self._items.pop()
        self._items[0].heap_index = 0
        self.sort_down(self._items[0])
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

def main():
    test_heap()

if __name__ == '__main__':
    main()