# Dustin Hartnett ENPM661 Project 1

import numpy as np
from queue import PriorityQueue

def Dijkstra(Startx, Starty, Goalx, Goaly):

  o = PriorityQueue()

  StartCoord = (Startx, Starty)

  GoalCoord = (Goalx, Goaly)

  o.put((0, 0, 0, StartCoord))

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

  # Set Node Index for next node visited to start counting
  nodeindex = 1

  # Define parent node for first node visited
  parentnodeindex = 0

  # Visited Nodes List
  visited = []

  # Begin BFS loop to explore nodal tree (aka run search while there are nodes to search)
  while len(o_pen) > 0:

    # Pop first node from open list
    now = o.get()

    # Add popped node to visited list
    visited.append(now)

    # End while loop if goal node is reached
    if now[3] == Goal:
      print('')
      print('Reached Goal State --- Updating Data Files')
      print('')
      print(now[3])
      print('')
      break
    
    currentNode = now.copy()
    nodeState = currentNode[3]

    [a, cost] = newNodes(nodeState)
    i = 0

    for maybeNode in a:

      if maybeNode not in visited:

        if inObstacle(maybeNode) == False:
          ctoc = calcCost(parentnodeindex, visited)
          o.put((ctoc+cost[i], nodeindex, parentnodeindex, maybeNode))
          nodeindex = nodeindex + 1
          i = i + 1
        
      elif maybeNode in visited:
        ctoc = calcCost(parentnodeindex, visited)
        h = 0

        for j in visited:
          k = j[3]

          if k == maybeNode:
            ogCost = k[0]

            if (ctoc+cost[i]) < ogCost:
              o.queue[h][0] == ctoc+cost[i]
              o.queue[h][1] == nodeindex
              o.queue[h][2] == parentnodeindex
          
          h = h +1

###---------------------------------------###

def newNodes(nodeState):

  node = nodeState.copy()
  x = node[0]
  y = node[1]
  a = []
  cost = []

  a.append((x+1,y)) # right
  cost.append(1)
  a.append((x-1,y)) # left
  cost.append(1)
  a.append((x,y+1)) # up
  cost.append(1)
  a.append((x,y-1)) # down
  cost.append(1)
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

def inObstacle(maybeNode):

  node = maybeNode.copy()
  vibes = False

  # If node in obstacle rages ......

  return vibes

###---------------------------------------###

def calcCost(parentindexnode, visited):

  case = False

  while case == False:

    for item in visited:

      parentindex = item[2]
      if parentindex == parentindexnode:

        parentcost = item[0]
        case = True
        #break need?????

###---------------------------------------###