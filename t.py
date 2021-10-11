from queue import PriorityQueue
import json

f = open('Dist.json',)
dist_json = json.load(f)
f = open('G.json',)
g_json = json.load(f)
f = open('Cost.json',)
e_json = json.load(f)

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

    def printPath(self, j):

        # Base Case : If j is source
        if self.parent[j] == -1:
            print("Shortest path: ", j, end=" "),
            return
        self.printPath(self.parent[j])
        print("->", j, end=" "),

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

print("start Computing")
D, E = g.dijkstra(1, 50)
g.printPath(50)

print("\nShortest distance: ", D[50])
print("Total energy cost: ", E[50])
