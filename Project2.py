import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt

def Dijkstra(Startx, Starty, Goalx, Goaly):

  # Priority Queue = Open List
  ope = PriorityQueue()

  # Dictionary = Closed List
  closed = {}

  openCheck = set()

  # Start Coordinates
  StartCoord = (Startx, Starty)

  # Goal Coordinates
  GoalCoord = (Goalx, Goaly)

  # Insert Starting node and information into open list
  # (Cost, Node Index, Parent Node Index, Node)
  ope.put((0.0, 0, 0, StartCoord))

  # Print Start Node
  print('')
  print('Starting Coordinate')
  print('')
  print(StartCoord)
  print('')
  print('Goal Coordinate')
  print('')
  print(GoalCoord)
  print('')
  print('')
  print('#-----Calculating Path To Goal Coordinate-----#')
  print('')

  nodeindex = 0

  # Begin BFS loop to explore nodal tree (aka run search while there are nodes to search)
  while ope.qsize() > 0:

    # check first node from open list
    now = ope.get(0) # (Cost, Node Index, Parent Node Index, Node)

    # Assign variable names to values in node pulled from open list
    currentNode = tuple(now)
    nodeState = currentNode[3]
    parentnodeindex = currentNode[1]
    parentcost = currentNode[0]

    # Add popped node to closed dictionary
    closed[nodeState] = parentcost, currentNode[1], currentNode[2], nodeState

    #nodeindex = nodeindex + 1

    # End while loop if goal node is reached
    if nodeState[0] == Goalx and nodeState[1] == Goaly:
      print('')
      print('Reached Goal State --- Updating Data Files')
      print('')
      print(now[3])
      print('')
      break

    # Derive new nodes and associated costs
    [a, cost] = newNodes(nodeState)

    # go through each new node
    for maybeNode,c in zip(a,cost):

      # Check to see if node is in closed list, if not then proceed
      if closed.get(maybeNode, None) == None:

        # Check to see if node is in free space, if it is then add it to the open list queue
        if inObstacle(maybeNode) == False:

          if maybeNode not in openCheck:

            '''
            print(' ')
            print('New Node')
            print(maybeNode)
            print('')
            '''
            nodeindex = nodeindex + 1
            ope.put((round((parentcost+c), 2), nodeindex, parentnodeindex, maybeNode))
            #print((round((parentcost+c), 2), nodeindex, parentnodeindex, maybeNode))
            openCheck.add(maybeNode)
            

      # if node is already in the closed list, compare costs and update values as necessary 
      else:
        '''
        print('')
        print('checking old node')
        print(maybeNode)
        print('')
        '''

        j = closed.get(maybeNode)
        
        if j[0] > (parentcost+c):
          nodeindex = nodeindex + 1
          #closed[maybeNode] = parentcost+c, j[1], parentnodeindex, j[3]
          closed[maybeNode] = parentcost+c, nodeindex, parentnodeindex, j[3]
  '''
  print(ope.queue)
  print('')
  print(closed)
  
  print(len(ope.queue))
  print('')
  print(len(closed))
  '''

  #print(closed)

  return closed
###---------------------------------------###

# Take current node from open list and make 8 new nodes and 8 new costs
def newNodes(nodeState):

  node = tuple(nodeState)
  x = node[0]
  y = node[1]
  a = []
  cost = []

  a.append((x+1,y)) # right
  cost.append(1.0)
  a.append((x-1,y)) # left
  cost.append(1.0)
  a.append((x,y+1)) # up
  cost.append(1.0)
  a.append((x,y-1)) # down
  cost.append(1.0)
  a.append((x+1,y+1)) # up right
  cost.append(1.4)
  a.append((x+1,y-1)) # down right
  cost.append(1.4)
  a.append((x-1,y+1)) # left up
  cost.append(1.4)
  a.append((x-1,y-1)) # left down
  cost.append(1.4)

  return a, cost

###---------------------------------------###

# Check to see if node in question lies within obstacle space
# Return 'False' if in free space, 'True' if in an obstacle or outside the boundaries
# These equations include the 5 space boundary around the obstacles  
def inObstacle(maybeNode):

  node = tuple(maybeNode)
  xnode = node[0]
  ynode = node[1]
  vibes = False

  # check if in map
  if xnode < 5 or xnode > 1195 or ynode < 5 or ynode > 495:
    vibes = True

  # check first obstacle (rectangle)
  elif xnode > 95 and xnode < 180 and ynode > 95:# and ynode <= 500:
    vibes = True

  # check second obstacle (rectangle)
  elif xnode > 270 and xnode < 355 and ynode < 405: # and ynode >= 0
    vibes = True

  # check third obstacle (hexagon)
  elif xnode > 515 and xnode < 785 and (0.556*xnode - ynode + 43.66 > 0) and (-0.556*xnode - ynode + 766.4 > 0) and (-0.556*xnode - ynode + 456.34 < 0) and (0.556*xnode - ynode - 266.4 < 0):
    vibes = True

# The next three compose the concave fourth obstacle
  elif xnode > 895 and xnode < 1025 and ynode > 370 and ynode < 455:
    vibes = True

  elif xnode > 895 and xnode < 1025 and ynode > 45 and ynode < 130:
    vibes = True
  
  elif xnode > 1015 and xnode < 1105 and ynode > 45 and ynode < 455:
    vibes = True

  # return "vibes". False = node is in free space. True = node is out of map or in obstacle.
  return vibes

###---------------------------------------###

# Back Track algorithm to find optimized path to goal
def BackTrack(closed, Startx, Starty, Goalx, Goaly):

  pathx = []

  pathy = []

  goalNode = (Goalx, Goaly)
  startNode = (Startx, Starty)

  goalInfo = closed.get(goalNode)
  #print('')
  #print(goalInfo)

  nodeState = goalInfo[3]
  parent = goalInfo[2]

  pathx.append(nodeState[0])
  pathy.append(nodeState[1])

  while nodeState != startNode:

    for key in closed:

      item = closed.get(key)
      coord = item[3]
      index = item[1]

      if index == parent:
        pathx.append(coord[0])
        pathy.append(coord[1])
        parent = item[2]
        nodeState = coord
        #print('')
        #print(nodeState)
        break

  pathx.reverse()
  pathy.reverse()

  return pathx, pathy


###---------------------------------------###

start = time.time()

#Dijkstra(635, 400, 665, 400)

Startx = 5

Starty = 5

Goalx = 700

Goaly = 50

closed = Dijkstra(Startx, Starty, Goalx, Goaly)

[pathx, pathy] = BackTrack(closed, Startx, Starty, Goalx, Goaly)

#print(Dijkstra(5, 5, 12, 12))

end = time.time()

print('')
print('Time in Seconds:')
print('')
print(end-start)
print('')

plt.plot(pathx, pathy, label='path') # plots T1 w/respect to time
plt.xlabel("x") # labels the plot's x-axis
plt.ylabel("y") # labels the plot's y-axis
plt.show()

'''
DICT1
key = (node, index, parent index)
value = cost

DICT2
key = node
value (index, parent index)

search dict2 for node
use (node, index, parent index) to search dict 1

remove both entries from both dictionaires

update both dictionaries - replace cost and parent index and index if lower'''