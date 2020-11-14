graph = {'A': {'B': 6.5, 'F': 2.2},
         'B': {'A': 6.5, 'C': 1.1, 'D': 4.2, 'E': 3.2},
         'C': {'B': 1.1, 'D': 1.6},
         'D': {'F': 0.7, 'B': 4.2, 'C': 1.6, 'E': 2.9},
         'E': {'B': 3.2, 'D': 2.9, 'F': 6.2},
         'F': {'A': 2.2, 'D': 0.7, 'E': 6.2},
}
unknown = 999


def run():
    global graph
    dijkstra(graph, 'A')


def dijkstra(graph,start):
    shortestDistances = {}
    prevNode = {}
    toVisit = graph.copy()

    for node in graph:
        shortestDistances[node] =  unknown
    shortestDistances[start] = 0

    while toVisit:
        closestNode = None

        # find the next closest node
        for node in toVisit:
            if closestNode is None:
                closestNode = node
            elif shortestDistances[node] < shortestDistances[closestNode]:
                closestNode = node

        for node, distance in graph[closestNode].items():
            if node in graph:
                newDistance = distance + shortestDistances[closestNode]
                if newDistance < shortestDistances[node]:
                    shortestDistances[node] = newDistance
                    prevNode[node] = closestNode
        toVisit.pop(closestNode)

    # Print out the shortest paths to each node
    nodes = list(graph.keys())
    nodes.remove(start)
    for node in nodes:
        path = []
        currentNode = node

        while currentNode != start:
            try:
                path.insert(0,currentNode)
                currentNode = prevNode[currentNode]
            except KeyError:
                print(f"Node {node} cannot be reached")
                break
        path.insert(0,start)
        path = ''.join(path)
        distance = str(round(shortestDistances[node], 1))
        if distance != str(unknown):
            print(f"Least cost path to router {node}:{path} and the cost is {distance}")

run()
