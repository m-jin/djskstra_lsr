import json
import sys
import threading
import time
from socket import *


ROUTE_UPDATE_INTERVAL = 30
UPDATE_INTERVAL = 1

filename = sys.argv[1]
host = 'localhost' #assume we are running all the nodes on the same computer
nodeArray = []
graph = {}
adjacentNodePorts = {}
heartbeats = {}
serverPort = 0
serverSocket = socket(AF_INET, SOCK_DGRAM)

def run():
    InitNode()

    broadcast = threading.Thread(target=BroadcastMessage, daemon=True)
    broadcast.start()

    getShortestPath = threading.Thread(target=GetShortestPath, daemon=True)
    getShortestPath.start()

    while True:
        ReceiveMessage()


def InitNode():
    """create "router"""
    global nodeArray
    global adjacentNodePorts

    nodeArray = FileToArray(filename)
    serverPort = nodeArray[0][1]
    initialiseHeartbeartsAndPorts()

    serverSocket.bind((host, serverPort))


def ReceiveMessage():
    """listen to port for messages"""
    global graph
    message, senderAddress = serverSocket.recvfrom(4096)
    decodedMessage = json.loads(message.decode("utf-8"))
    SaveNodeHeartbeat(decodedMessage)
    graph.update(decodedMessage)

    portsToSendTo = GetPortsToUse(decodedMessage, senderAddress[1])
    node = list(decodedMessage.keys())[0]
    if bool(decodedMessage[node]):
        for port in portsToSendTo:
            forwardPacket = threading.Thread(target=SendMessage, args=[message, port])
            forwardPacket.start()


def GetPortsToUse(message, senderPort):
    """decide which ports to forward broadcast to.
    Do not send back to sender or any routers sender
    already broadcasted to"""
    childNodes = list(message.values())[0]
    activeAdjacentNodes = list(GetPaths().keys())
    portsToUse = []

    for node in activeAdjacentNodes:
        if node not in childNodes and adjacentNodePorts[node] != senderPort:
            portsToUse.append(adjacentNodePorts[node])
    return portsToUse

def SendMessage(message, port):
    """used in self broadcasting and broadcast forwarding"""
    address = (host, port)
    serverSocket.sendto(message, address)


def BroadcastMessage():
    global graph

    while True:
        nodeDict = NodeArrayToDict()
        graph.update(nodeDict)
        message = json.dumps(nodeDict).encode('utf-8')
        for port in adjacentNodePorts.values():
            SendMessage(message, port)
        time.sleep(UPDATE_INTERVAL)


def FileToArray(filename):
    """turn txt file to 2D array"""
    array = []
    with open(filename) as f:
        for line in f:
            line = [CleanString(string) for string in line.strip('\n').split()]
            array.append(line)
    return array


def CleanString(string):
    """sort string into int or float if needed"""
    if "." in string:
        string = float(string)
    else :
        try:
            string = int(string)
        except:
            pass
    return string


def initialiseHeartbeartsAndPorts():
    global nodeArray
    global adjacentNodePorts

    for array in nodeArray:
        if len(array) == 3:
            adjacentNodePorts[array[0]] = array[2]
            heartbeats[array[0]] = 0


def NodeArrayToDict():
    global nodeArray
    nodeDict = {}

    nodeDict[nodeArray[0][0]] = GetPaths()
    return nodeDict


def GetPaths():
    global nodeArray
    paths = {}

    for array in nodeArray:
        if len(array) == 3:
            if(IsNodeAlive(array[0])):
                paths[array[0]] = array[1]
    return paths


def GetDijkstraPath():
    start = nodeArray[0][0]
    unknown = 999
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
    print(f"I am router {nodeArray[0][0]}")
    for node in nodes:
        path = []
        currentNode = node

        while currentNode != start:
            try:
                path.insert(0,currentNode)
                currentNode = prevNode[currentNode]
            except KeyError:
                break
        path.insert(0,start)
        path = ''.join(path)
        distance = str(round(shortestDistances[node], 1))
        if distance != str(unknown):
            print(f"Least cost path to router {node}: {path} and the cost is {distance}")


def GetShortestPath():
    while True:
        GetDijkstraPath()
        time.sleep(ROUTE_UPDATE_INTERVAL)


def SaveNodeHeartbeat(message):
    node = list(message.keys())[0]
    if node in adjacentNodePorts:
        timestamp = time.time()
        heartbeats[node] = timestamp


def IsNodeAlive(node):
    """remove dead nodes from broadcast message"""
    if time.time() - heartbeats[node] > 4:
        return False
    return True


run()
