from Tkinter import *
import struct
import xml.etree.ElementTree as ET
from Queue import *
import math

MPERLAT = 111000
MPERLON = MPERLAT * math.cos(42*math.pi/180)
GRIDSIZE = 0

class Node():
    #Node(id,pos,ways,elvel,wayinfo)
    def __init__(self,id,p,e=0):
        self.id = id
        self.pos = p
        self.ways = []
        self.elev = e

class Cost():
    def __init__(self,ways,begin,end):
        self.wayz = ways
        self.begin = begin
        self.end = end
        self.cost = node_dist(begin,end)
        #uphill to downhill
        if (begin.elev - end.elev) > 1:
            self.cost *= 1.2
        elif (begin.elev - end.elev) < 1:
            self.cost *= 2
        else:
            self.cost *= 1

class pathWay():
    def __init__(self, wayName):
        self.wayName = wayName
        self.WPs = []


class pathFinding():

    def __init__(self,nodes,ways):
        self.nodes = nodes
        self.path = ways

    def heur(self,start, fixend):
        #cloeset distacen between two node is the stright line dintance between two node.
        return node_dist(start,fixend)

    def astar(self, starts, dests):
        open = {}
        cost = {}
        frontier = PriorityQueue()
        init_cost = self.heur(nodes[starts], nodes[dests])
        frontier.put(starts,init_cost)
        open[starts] = None
        cost[starts] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == dests:
                print ("path find and the cost is: ", cost[dests]*60/100000, "hours")
                return self.display_path (open,dests)

            for way in nodes[current].ways:
                cost_update = cost[current] + way.cost

                #check if it is in the open list
                #check if new cost is better
                if way.end.id not in open or cost_update < cost[way.end.id]:
                    nway_id = way.end.id
                    cost[nway_id] = cost_update
                    f = cost_update + self.heur(nodes[nway_id], nodes[dests])
                    frontier.put(nway_id,f)
                    open[way.end.id] = (current,way.wayz)

           # for i in current.ways

    def display_path(self,open,dests):
        path = []
        for i in open:
            name = self.search_name(i)

            if name not in path:
                path.append(name)

        print "final path between the starting and ending position is: ",path


    def search_name(self,id):

        #print dests
        found = False
        temp = []
        for i in ways:
            for j in ways[i].WPs:
                if j == id:
                    temp.append(ways[i].wayName)
        return temp[0]

class datahandle():

    def read_elevations(self,map):
        efile = open(map)
        estr = efile.read()
        elevations = []
        for spot in range(0, len(estr), 2):
            elevations.append(struct.unpack('>h', estr[spot:spot + 2])[0]) #2D -> 1D. each row contains x + yx, from x=0,y=0, at 43,79 as 0,0)
        GRIDSIZE = (int) (math.sqrt(len(elevations)))

        return elevations,GRIDSIZE

    def read_xml(self,elevations,GRIDSIZE):
        '''
        This function reads in a piece of OpenStreetMap XML and prints
        out all of the names of all of the "ways" (linear features).
        You should replace the print with something that adds the way
        to a data structure useful for searching.
        '''
        tree = ET.parse('test.osm')
        root = tree.getroot()
        nodes = dict()
        ways = dict()
        for item in root:
            #find all points
            if item.tag == 'node':
                #find all position on the map
                nodeId = (long) (item.get('id'))
                position = ((float)(item.get('lat')), (float)(item.get('lon')))
                row_cal = (int)((44 - position[0]) * GRIDSIZE)
                col_cal = (int)((79 - position[1]) * GRIDSIZE)
                elevation = elevations[row_cal * GRIDSIZE + col_cal]
                # Node(id,pos,ways,elvel,wayinfo)
                nodes[nodeId] = Node(nodeId,position,elevation)
            elif item.tag == 'way':
                getway = False
                wayName = None
                oneway = False

                for subitem in item:

                    #need to check for both name of the road, name of the road, and direction of the road
                    if subitem.tag == 'tag' and subitem.get('k') == 'highway':
                        getway = True

                    if subitem.tag == 'tag' and subitem.get('k') == 'name':
                        wayName = subitem.get('v')

                    if subitem.tag == 'tag' and subitem.get('k') == 'oneway':
                        if subitem.get('v') == 'yes':
                            oneway = True

                if getway == True:
                    way_point = (long)(item.get('id'))
                    if wayName == None:
                        wayName = "unknown"
                    ways[way_point] = pathWay(wayName)
                    wayList = []
                    for subitem in item:
                        if subitem.tag == 'nd':
                            wayList.append((long)(subitem.get('ref')))

                    #making connections
                    initWay = wayList[0]
                    for i in range (len(wayList)-1):
                        nextWay = wayList[i+1]
                        nodes[initWay].ways.append(Cost(ways[way_point],nodes[initWay],nodes[nextWay]))
                        initWay = nextWay

                    if oneway == False:
                        initWay = wayList[-1]
                        for j in range (len(wayList)-2,-1,-1):
                            nextWay = wayList[j]
                            nodes[initWay].ways.append(Cost(ways[way_point],nodes[initWay], nodes[nextWay]))
                            initWay = nextWay
                    ways[way_point].WPs = wayList

        return nodes,ways

    def findID(self,name,ways):
        for i in ways:
            if ways[i].wayName == name:
                node = ways[i].WPs[0]
                return node



def node_dist(n1, n2):
    # to find distance between two point
    dx = (n2.pos[0] - n1.pos[0]) * MPERLON
    dy = (n2.pos[1] - n1.pos[1]) * MPERLAT
    return math.sqrt(dx * dx + dy * dy)




data = datahandle()
elevations, gridsize = data.read_elevations("n43_w079_3arc_v2.bil")
nodes,ways = data.read_xml(elevations,gridsize)
startNode = data.findID("Marquette Avenue",ways)
endNode = data.findID("Fernhill Boulevard",ways)

pathFinding(nodes,ways).astar(startNode,endNode)
