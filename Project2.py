import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt
import cv2
import os


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
          closed[maybeNode] = parentcost+c, nodeindex, parentnodeindex, j[3]

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
        break

  pathx.reverse()
  pathy.reverse()

  return pathx, pathy


###---------------------------------------###

start = time.time()

Startx = 5

Starty = 5

Goalx = 200

Goaly = 50

closed = Dijkstra(Startx, Starty, Goalx, Goaly)

[pathx, pathy] = BackTrack(closed, Startx, Starty, Goalx, Goaly)

end = time.time()

print('')
print('Time in Seconds:')
print('')
print(end-start)
print('')

###---------------------------------------###

# Plot It All! 

green = (31, 80, 12)
blue = (255, 0, 0)
red = (0, 0, 255)
orange = (0, 105, 30)
yellow = (255,0,205)
white = (255, 255, 255)

map = np.ones((500,1200,3), dtype=np.uint8) # blank map
map = 255*map # make it white

# First Obstacle
#cv2.rectangle(map, (100,500), (175,100), green, 1)
cv2.rectangle(map, (100,0), (175,400), green, -1)

# Second Obstacle
cv2.rectangle(map, (275,100), (350,500), green, -1)

# Third Obstacle
hexpts = np.array([[520, 325], [650, 400], [780, 325], [780, 175], [650, 100], [520, 175]])
hexpts = hexpts.reshape(-1,1,2)
cv2.fillPoly(map, [hexpts], green)

# Fourth Obstacle
concavepts = np.array([[900, 450], [1100, 450], [1100, 50], [900, 50], [900, 125], [1020, 125], [1020, 375], [900, 375]])
concavepts = concavepts.reshape(-1,1,2)
cv2.fillPoly(map, [concavepts], green)

# 5 pixel buffer

# First Obstacle
cv2.rectangle(map, (95,5), (180,405), red, 3)

# Second Obstacle
cv2.rectangle(map, (270,500), (355,95), red, 3)

# Third Obstacle
hexpts = np.array([[515, 330], [650, 405], [785, 330], [785, 170], [650, 95], [515, 170]])
hexpts = hexpts.reshape(-1,1,2)
cv2.polylines(map, [hexpts], isClosed=True, thickness=3, color=red)

# Fourth Obstacle
concavepts = np.array([[895, 455], [1105, 455], [1105, 45], [895, 45], [895, 130], [1015, 130], [1015, 370], [895, 370]])
concavepts = concavepts.reshape(-1,1,2)
cv2.polylines(map, [concavepts], isClosed=True, thickness=3, color=red)

# map border
cv2.rectangle(map, (0,0), (1200,500), yellow, 3)

# map border + 5 pixel buffer
cv2.rectangle(map, (5,495), (1195, 5), (0,0,0), 1)

# empty image array to store frames

image_array = []

# loop through closed coordinates and plot one by one on the map
for key in closed:
  information = closed.get(key)
  coord = information[3]
  x = coord[0]
  y = 500-coord[1]
  cv2.drawMarker(map, (x,y), color = [0, 165, 255], thickness=2, markerType= cv2.MARKER_SQUARE, line_type=cv2.LINE_AA, markerSize=1)
  #cv2.imshow("map", map)
  #print('here')

  # Trying to take images of the map for each iteration and add image to an array ... it's not working
  image = pyautogui.screenshot()
  image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
  height, width, layers = image.shape
  size = (width,height)
  image_array.append(image)

# loop through optimized Path coordinates and plot one by one on the map

for x1,y1 in zip(pathx,pathy):
  cv2.drawMarker(map, (x1,500-y1), color = [0, 0, 0], thickness=2, markerType= cv2.MARKER_SQUARE, line_type=cv2.LINE_AA, markerSize=1)

  # Trying to take images of the map for each iteration and add image to an array ... it's not working
  image = pyautogui.screenshot()
  image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
  height, width, layers = image.shape
  size = (width,height)
  image_array.append(image)

# Define video file
out = cv2.VideoWriter(cv2.VideoWriter('scenario1.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size))
 
# Add frames to video ... also not working
for i in range(len(image_array)):
  out.write(image_array[i])
out.release()

#cv2.imshow("map", map)
#cv2.waitKey(0)

###---------------------------------------###

'''

NOTES FOR FURTURE IMPLIMENTATION


plt.plot(pathx, pathy, label='path') # plots T1 w/respect to time
plt.xlabel("x") # labels the plot's x-axis
plt.ylabel("y") # labels the plot's y-axis
plt.show()
'''
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