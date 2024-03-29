# Course: CS261 - Data Structures
# Author: Adam Okasha
# Assignment: 6
# Description: Graphs

from collections import deque

class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Add new vertex to the graph
        """
        if v in self.adj_list:
            return
        self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Add edge to the graph
        """
        if u == v:
            return
        if not u in self.adj_list:
            self.adj_list[u] = [v]
        else:
            if v not in self.adj_list[u]:
                self.adj_list[u].append(v)
        if not v in self.adj_list:
            self.adj_list[v] = [u]
        if u not in self.adj_list[v]:
            self.adj_list[v].append(u)

    def remove_edge(self, v: str, u: str) -> None:
        """
        Remove edge from the graph
        """
        if v in self.adj_list and u in self.adj_list[v]:
            self.adj_list[v].remove(u)
        if u in self.adj_list and v in self.adj_list[u]:
            self.adj_list[u].remove(v)

    def remove_vertex(self, v: str) -> None:
        """
        Remove vertex and all connected edges
        """
        if v in self.adj_list:
            del self.adj_list[v]
        for k in self.adj_list:
            if v in self.adj_list[k]:
                self.adj_list[k].remove(v)
        

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        vertices = []
        for v in self.adj_list:
            vertices.append(v)
        return vertices

       

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)
        """
        all_edges = []
        for k in self.adj_list:
            for e in self.adj_list[k]:
                if (e, k) not in all_edges:
                    all_edges.append((k,e))
        return all_edges
        

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        if not len(path):
            return True
        v = path.pop()
        if v not in self.adj_list:
            return False
        adj_vertices = self.adj_list[v]
        while len(path):
            v = path.pop()
            if v not in adj_vertices:
                return False
            adj_vertices = self.adj_list[v]
        return True
       
    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        if v_start not in self.adj_list:
            return []
        visited = []
        stack = [v_start]
        while len(stack):
            v = stack.pop()
            if v not in visited:
                visited.append(v)
                successors = self.adj_list[v]
                if successors:
                    successors.sort()
                    successors.reverse()
                    stack = stack + successors
            if v == v_end:
                break
        return visited
            
    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        if v_start not in self.adj_list:
            return []
        visited = []
        queue = deque([v_start])
        while len(queue):
            v = queue.popleft()
            if v not in visited:
                visited.append(v)
                successors = self.adj_list[v]
                if successors:
                    successors.sort()
                    for successor in successors:
                        queue.append(successor)
            if v == v_end:
                break
        return visited

    def count_connected_components(self):
        """
        Return number of connected componets in the graph
        """
        vertices = self.adj_list.keys()
        if not len(vertices):
            return 0
        connected_count = 0
        visited_map = {}
        for v in vertices:
            if v not in visited_map:
                path = self.dfs(v)
                for u in path:
                    visited_map[u] = 1
                connected_count += 1
        return connected_count

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        vertices = self.adj_list.keys()
        status = {}
        for k in self.adj_list.keys():
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
        This pseudo code solution however did not work for me. For example the condition u != parent(v)
        would return True here in all cases. I modified it to disregard the iteration when
        u == parent(v) and call itself recursively when status of a neighbour is not 'VISITED' and added
        a catch all of returning False.

        Args:
            v (object): current/anchor node
            parent (object): parent node
            status (object): holds the status of all vertices: 'NOT_VISITED', 'VISITING', 'VISITED' 

        Returns:
            bool: True if there is a cycle, False if not
        """
        status[v] = 'VISITING'
        for u in self.adj_list[v]:
            if u == parent:
                continue
            if status[u] == 'VISITING':
                return True
            elif status[u] != 'VISITED' and self._dfs_has_cycle(u, v, status):
                return True
        
        status[v] = 'VISITED'
        return False


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
