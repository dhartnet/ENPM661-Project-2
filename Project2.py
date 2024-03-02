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
  while o.qsize() > 0:

    # Pop first node from open list
    now = o.get()

    # Add popped node to visited list
    visited.append(now)

    currentNode = tuple(now)
    nodeState = currentNode[3]

    # End while loop if goal node is reached
    if nodeState[0] == Goalx and nodeState[1] == Goaly:
      print('')
      print('Reached Goal State --- Updating Data Files')
      print('')
      print(now[3])
      print('')
      break
    
    [a, cost] = newNodes(nodeState)
    i = 0

    for maybeNode in a:

      if maybeNode not in visited:

        if inObstacle(maybeNode) == False:
          print(maybeNode)
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
              #print(maybeNode)
              o.queue[h][0] == ctoc+cost[i]
              o.queue[h][1] == nodeindex
              o.queue[h][2] == parentnodeindex
          
          h = h +1
        i = i + 1

###---------------------------------------###

def newNodes(nodeState):

  node = tuple(nodeState)
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

  node = tuple(maybeNode)
  xnode = node[0]
  ynode = node[1]
  vibes = False

  # check if in map
  if xnode < 5 and xnode > 1195 and ynode < 5 and ynode > 495:
    vibes = True

  # check first obstacle (rectangle)
  elif xnode > 95 and xnode < 180 and ynode > 95 and ynode <= 500:
    vibes = True

  # check second obstacle (rectangle)
  elif xnode > 270 and xnode < 355 and ynode >= 0 and ynode < 405:
    vibes = True

  # check third obstacle (hexagon)
  elif xnode > 515 and xnode < 785 and ((15/26)*xnode - ynode + 325) <= 0 and ((-15/26)*xnode - ynode + 400) <= 0 and ((-15/26)*xnode - ynode + 175) >= 0 and ((15/26)*xnode - ynode + 100) >= 0:
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

def calcCost(parentindexnode, visited):

  case = False

  while case == False:

    for item in visited:

      parentindex = item[2]
      if parentindex == parentindexnode:

        parentcost = item[0]
        case = True
        break

  return parentcost
###---------------------------------------###

Dijkstra(5, 5, 9, 9)
print('hell yeah')