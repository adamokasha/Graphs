# Course: CS261 - Data Structures
# Author: Adam Okasha
# Assignment: 6
# Description: Graphs

import heapq
from collections import deque

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        TODO: Write this implementation
        """
        self.v_count += 1
        if not len(self.adj_matrix):
            self.adj_matrix.append([0])
            return self.v_count
        new_row = []
        for i in range(self.v_count):
            new_row.append(0)
        self.adj_matrix.append(new_row)
        for i in range(self.v_count - 1):
            self.adj_matrix[i].append(0)
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        TODO: Write this implementation
        """
        l = len(self.adj_matrix[0])
        if weight < 1 or src < 0 or src > l - 1 or dst < 0 or dst > l - 1 or src == dst:
            return
        self.adj_matrix[src][dst] = weight 

    def remove_edge(self, src: int, dst: int) -> None:
        """
        TODO: Write this implementation
        """
        l = len(self.adj_matrix[0])
        if src < 0 or src > l - 1 or dst < 0 or dst > l - 1 or src == dst:
            return
        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        TODO: Write this implementation
        """
        if not len(self.adj_matrix):
            return []
        vertices = []
        for i in range(len(self.adj_matrix[0])):
            vertices.append(i)
        return vertices


    def get_edges(self) -> []:
        """
        TODO: Write this implementation
        """
        edges = []
        for i in range(self.v_count):
            for j in range(self.v_count):
                if self.adj_matrix[i][j] != 0:
                    edges.append((i, j, self.adj_matrix[i][j]))
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        TODO: Write this implementation
        """
        for i in range(len(path)):
            current_step = path[i]
            next_step = path[i + 1] if i + 1 < len(path) else None
            if next_step:
                weight = self.adj_matrix[current_step][next_step]
                if weight == 0:
                    return False
        return True



    # def dfs(self, v_start, v_end=None) -> []:
    #     """
    #     TODO: Write this implementation
    #     """
    #     pass

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        if v_start > len(self.adj_matrix[0]):
            return []
        visited = []
        stack = [v_start]
        while len(stack):
            v = stack.pop()
            if v not in visited:
                visited.append(v)
                successors = self._get_adj_vertices(v)
                if successors:
                    successors.sort()
                    successors.reverse()
                    stack = stack + successors
            if v == v_end:
                break
        return visited

    def _get_adj_vertices(self, v):
        adj_v_l = []
        for j in range(len(self.adj_matrix[v])):
            if self.adj_matrix[v][j] != 0:
                adj_v_l.append(j)
        return adj_v_l

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        if v_start > len(self.adj_matrix[0]):
            return []
        visited = []
        queue = deque([v_start])
        while len(queue):
            v = queue.popleft()
            if v not in visited:
                visited.append(v)
                successors = self._get_adj_vertices(v)
                if successors:
                    successors.sort()
                    for successor in successors:
                        queue.append(successor)
            if v == v_end:
                break
        return visited

    # def has_cycle(self):
    #     """
    #     TODO: Write this implementation
    #     """
    #     pass

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        vertices = []
        for i in range(self.v_count):
            vertices.append(i)
        status = {}
        for k in vertices:
            status[k] = 'NOT_VISITED'
        for v in vertices:
            _has_cycle = self._dfs_has_cycle(v, None, status)
            if _has_cycle:
                return True
        return False

    def _dfs_has_cycle(self, v, parent, status):
        """Recursive has_cycle helper returns True when a the neighbouring
        node is being visited and the anchor node is in the same state while
        not being the same as the parent node. This condition means that there is
        a cycle. 

        Note: This is very loosely based on this solution linked here:
        https://www.baeldung.com/cs/cycles-undirected-graph#cycle-detection
        The expanation and modifications used can be found in ud_graph.py comments.
        The difference between the undirected graph and the directed graph implementation
        is that two vertices pointing to each other can be considered a cycle.

        Args:
            v (object): current/anchor node
            parent (object): parent node
            status (object): holds the status of all vertices: 'NOT_VISITED', 'VISITING', 'VISITED' 

        Returns:
            bool: True if there is a cycle, False if not
        """
        status[v] = 'VISITING'
        adj_vertices = self._get_adj_vertices(v)
        for u in adj_vertices:
            if status[u] == 'VISITING':
                return True
            elif status[u] != 'VISITED' and self._dfs_has_cycle(u, v, status):
                return True
        
        status[v] = 'VISITED'
        return False

    def dijkstra(self, src: int) -> []:
        """
        TODO: Write this implementation
        """
        visited_map = {}
        priority_q = []
        shortest_path = []
        heapq.heapify(priority_q)
        heapq.heappush(priority_q, (0, 0, src))
        while len(priority_q):
            _, d, v = heapq.heappop(priority_q)
            if v not in visited_map:
                visited_map[v] = d
                for i in range(self.v_count):
                    s_d = self.adj_matrix[v][i]
                    if s_d != 0:
                        heapq.heappush(priority_q, (d + s_d, d + s_d, i))
                    else:
                        heapq.heappush(priority_q, (float('inf'), float('inf'), i))
        for i in range(self.v_count):
            shortest_path.append(visited_map[i])
        return shortest_path


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)

    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')

    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))

    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')

    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)

    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
