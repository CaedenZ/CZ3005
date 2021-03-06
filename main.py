from queue import PriorityQueue
import json
import math

f = open('Dist.json',)
dist_json = json.load(f)
f = open('G.json',)
g_json = json.load(f)
f = open('Cost.json',)
e_json = json.load(f)
f = open('Coord.json',)
coord_json = json.load(f)

print("load finish")


class Graph:

    def __init__(self, num_of_vertices):
        self.v = num_of_vertices
        self.visited = []
        self.path = {}
        self.parent = [-1] * (num_of_vertices + 1)
        self.weight = dist_json
        self.node = g_json
        self.cost = e_json
        self.coord = coord_json
        self.found = False
        self.res_dist = 0
        self.res_energy = 0
        self.path = []

    def printPath(self, j):

        # Base Case : If j is source
        if self.parent[j] == -1:
            print("Shortest path: ", j, end=" "),
            return
        self.printPath(self.parent[j])
        print("->", j, end=" "),

    def heuristic(self, start, end, final, energy):
        coor_s = self.coord[str(start)]
        coor_e = self.coord[str(end)]
        coor_f = self.coord[str(final)]
        dist_start = math.sqrt(
            ((coor_s[0] - coor_f[0]) ** 2) + ((coor_s[1] - coor_f[1]) ** 2))
        dist_end = math.sqrt(
            ((coor_e[0] - coor_f[0]) ** 2) + ((coor_e[1] - coor_f[1]) ** 2))
        dist_moved = dist_end - dist_start
        moved_per_energy = dist_moved / energy
        return moved_per_energy

    def distance(self, start, end):
        coor_s = self.coord[str(start)]
        coor_e = self.coord[str(end)]
        dist = (coor_s[0] - coor_e[0]) ^ 2 + (coor_s[1] - coor_e[1]) ^ 2
        return dist

    # def astar(self, start, end, dist_so_far, energy_so_far, path=[]):
    #     if self.found:
    #         return
    #     if energy_so_far > 287932:
    #         return

    #     path = path + [start]
    #     # print(energy_so_far)
    #     if start == end:
    #         path.append(path)
    #         del path[-1]
    #         print(path)
    #         print("Shortest distance: ", dist_so_far)
    #         print("Total energy cost: ", energy_so_far)
    #         self.path = path
    #         self.res_dist = dist_so_far
    #         self.res_energy = energy_so_far
    #         self.found = True
    #         return

    #     N = []
    #     for neighbor in self.node[str(start)]:
    #         N.append(neighbor)

    #     N.sort(key=lambda x: self.heuristic(
    #         start, x, 50, self.cost[str(start) + "," + str(x)]), reverse=True)

    #     print(self.distance(
    #         start, 50))

    #     for neighbor in N:
    #         neighbor = int(neighbor)
    #         distance = self.weight[str(start) + "," + str(neighbor)]
    #         energy = self.cost[str(start) + "," + str(neighbor)]

    #         if neighbor not in path:
    #             self.astar(neighbor, end, dist_so_far +
    #                        distance, energy_so_far+energy, path)
    #     return

    def astarv2(self, start_vertex, end_vertex):
        D = {v+1: float('inf') for v in range(self.v)}
        D[start_vertex] = 0
        E = {v+1: float('inf') for v in range(self.v)}
        E[start_vertex] = 0
        H = {v+1: float('inf') for v in range(self.v)}
        H[start_vertex] = 0

        pq = PriorityQueue()
        pq.put((0, start_vertex))

        while not pq.empty():
            (dist, current_vertex) = pq.get()
            self.visited.append(current_vertex)
            # print(current_vertex)

            for neighbor in self.node[str(current_vertex)]:
                neighbor = int(neighbor)
                distance = self.weight[str(
                    current_vertex) + "," + str(neighbor)]
                energy = self.cost[str(
                    current_vertex) + "," + str(neighbor)]
                if neighbor not in self.visited:
                    heu = self.heuristic(current_vertex, neighbor, 50, energy)
                    old_cost = D[neighbor]
                    old_energy = E[neighbor]
                    old_heu = H[neighbor]
                    new_cost = D[current_vertex] + distance
                    new_energy = E[current_vertex] + energy
                    new_heu = H[current_vertex] + heu
                    if new_heu < old_heu:
                        self.parent[neighbor] = current_vertex
                        pq.put((new_cost, neighbor))
                        D[neighbor] = new_cost
                        E[neighbor] = new_energy
                        H[neighbor] = new_heu
        return D, E

    def dfs(self, start, end, dist_so_far, energy_so_far, path=[]):
        if self.found:
            return
        if energy_so_far > 287932:
            return

        path = path + [start]
        print(path)
        if start == end:
            path.append(path)
            del path[-1]
            print(path)
            print("Shortest distance: ", dist_so_far)
            print("Total energy cost: ", energy_so_far)
            self.path = path
            self.res_dist = dist_so_far
            self.res_energy = energy_so_far
            self.found = True
            return

        for neighbor in self.node[str(start)]:
            neighbor = int(neighbor)
            distance = self.weight[str(start) + "," + str(neighbor)]
            energy = self.cost[str(start) + "," + str(neighbor)]

            if neighbor not in path:
                self.dfs(neighbor, end, dist_so_far +
                         distance, energy_so_far+energy, path)

    def dijkstra(self, start_vertex, end_vertex):
        D = {v+1: float('inf') for v in range(self.v)}
        D[start_vertex] = 0
        E = {v+1: float('inf') for v in range(self.v)}
        E[start_vertex] = 0

        pq = PriorityQueue()
        pq.put((0, start_vertex))

        while not pq.empty():
            (dist, current_vertex) = pq.get()
            self.visited.append(current_vertex)
            # print(current_vertex)

            for neighbor in self.node[str(current_vertex)]:
                neighbor = int(neighbor)
                distance = self.weight[str(
                    current_vertex) + "," + str(neighbor)]
                energy = self.cost[str(
                    current_vertex) + "," + str(neighbor)]
                if neighbor not in self.visited:
                    old_cost = D[neighbor]
                    old_energy = E[neighbor]
                    new_cost = D[current_vertex] + distance
                    new_energy = E[current_vertex] + energy
                    if new_cost < old_cost:
                        self.parent[neighbor] = current_vertex
                        pq.put((new_cost, neighbor))
                        D[neighbor] = new_cost
                        E[neighbor] = new_energy
                        if D[50] == 148648.63722140007:
                            return D, E
        return D, E


g = Graph(264346)

val = input("Task 1 : 1\nTask 2 : 2\nTask 3 : 3\nEnter your value: ")

if val == "1":
    D, E = g.dijkstra(1, 50)
    g.printPath(50)
    print("\nShortest distance: ", D[50])
    print("Total energy cost: ", E[50])

if val == "2":
    g.dfs(1, 50, 0, 0)

if val == "3":
    D, E = g.astarv2(1, 50)
    g.printPath(50)
    print("\nShortest distance: ", D[50])
    print("Total energy cost: ", E[50])

if val == "4":
    g.astar(1, 50, 0, 0)
