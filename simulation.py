
# Simulation of approximate SLC in 2D Euclidean space


import sys
import random
import math


class Graph():
    def __init__(self, n):
        self.V = n
        self.vertices =[]
        self.graph = [[0 for column in range(n)]
                      for row in range(n)]
        self.eps = 0

    def generate_graph(self, n):
        self.vertices = []
        self.graph = [[0 for column in range(n)]
                      for row in range(n)]
        self.eps = random.uniform(0, 3)

        for i in range(n):
            point = (random.uniform(-10, 10), random.uniform(-10, 10))
            self.vertices.append(point)

        for i in range(n):
            for j in range(n):
                self.graph[i][j] = math.sqrt( (self.vertices[i][0] - self.vertices[j][0])**2 + (self.vertices[i][1] -
                                                                                                self.vertices[j][
                                                                                                    1])**2 )
        #print(self.vertices)
        #print(self.graph)
        #print('epsilon = ', self.eps)

    # A utility function to print
    # the constructed MST stored in parent[]
    def printMST(self, parent):
        print("Edge \tWeight")
        total = 0
        for i in range(1, self.V):
            print(parent[i], "-", i, "\t", self.graph[i][parent[i]])
            total += self.graph[i][parent[i]]
        print("weight: ", total, "\n")
        return total

    def SLC(self):
        exactMSTParent = self.primMST()
        approxMSTParent = self.approxPrimMST()

        #print()
        #print('EXACT MST: ')
        # exact_weight = self.printMST(exactMSTParent)
        #print('APPROX MST')
        # approx_weight = self.printMST(approxMSTParent)

        #print("weight ratio: ", approx_weight / exact_weight, '\n')

        # EXACT SLC
        exactSLCCost = 0

        for i in range(1, self.V):
            exactSLCCost = max(exactSLCCost, self.graph[i][exactMSTParent[i]])

        # APPROX SLC
        largestEdge = 0
        edgeRemove = None

        for i in range(1, self.V):
            if self.graph[i][approxMSTParent[i]] > largestEdge:
                largestEdge = self.graph[i][approxMSTParent[i]]
                edgeRemove = i

        tree = [ [] for i in range(self.V) ]
        for i in range(1, self.V):
            if i != edgeRemove:
                tree[i].append(approxMSTParent[i])
                tree[approxMSTParent[i]].append(i)

        components = self.cc(tree)
        #print(components)
        c1 = components[0]
        c2 = components[1]

        approxSLCCost = sys.maxsize

        for u in c1:
            for v in c2:
                approxSLCCost = min(approxSLCCost, self.graph[u][v])

        # FINAL STATS
        ratio = exactSLCCost / approxSLCCost
        #print(exactSLCCost, approxSLCCost, 'ratio: ', ratio)
        # print(ratio, 1 + self.eps)
        if ratio > 1 + self.eps:
            print('Found Counterexample!')

            print(self.vertices)
            print(self.graph)
            print('epsilon = ', self.eps)

            print()
            print('EXACT MST: ')
            exact_weight = self.printMST(exactMSTParent)
            print('APPROX MST')
            approx_weight = self.printMST(approxMSTParent)

            print("weight ratio: ", approx_weight / exact_weight, '\n')

            print("Removed edge ", edgeRemove)
            print("components after removal ", components)
            print("exactSLCCost: ", exactSLCCost, "approxSLCCost: ", approxSLCCost, 'ratio: ', ratio, "\n")


    def cc(self, tree):
        visited = []
        components = []

        for i in range(self.V):
            if i not in visited:
                q = [i]
                visited.append(i)
                component = [i]

                while len(q) != 0:
                    curr = q.pop(0)
                    for neighbor in tree[curr]:
                        if neighbor not in visited:
                            q.append(neighbor)
                            visited.append(neighbor)
                            component.append(neighbor)

                components.append(component)
        return components



    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minKey(self, key, mstSet):

        # Initialize min value
        min = sys.maxsize

        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v

        return min_index

    # Function to construct and print MST for a graph
    # represented using adjacency matrix representation
    def primMST(self):
        # Key values used to pick minimum weight edge in cut
        key = [sys.maxsize] * self.V
        parent = [None] * self.V # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V

        parent[0] = -1 # First node is always the root of

        for cout in range(self.V):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minKey(key, mstSet)

            # Put the minimum distance vertex in
            # the shortest path tree
            mstSet[u] = True

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.V):

                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        return parent

    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def approxMinKey(self, key, mstSet):

        # Initialize min value
        min = sys.maxsize

        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]

        approx_edges = []
        for v in range(self.V):
            if key[v] <= (1 + self.eps) * min and mstSet[v] == False:
                approx_edges.append(v)
        return random.choice(approx_edges)


    # Function to construct and print MST for a graph
    # represented using adjacency matrix representation
    def approxPrimMST(self):
        # Key values used to pick minimum weight edge in cut
        key = [sys.maxsize] * self.V
        parent = [None] * self.V  # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V

        parent[0] = -1  # First node is always the root of

        for cout in range(self.V):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.approxMinKey(key, mstSet)

            # Put the minimum distance vertex in
            # the shortest path tree
            mstSet[u] = True

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.V):

                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        return parent


# Driver's code
if __name__ == '__main__':

    for iter in range(10000):
        g = Graph(6)
        g.generate_graph(6)

        g.SLC()

