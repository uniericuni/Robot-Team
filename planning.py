import heapq as hq
from config import *
from utility import *
# linked node to connect node with its parent and remember pos
class Node:
    def __init__(self, parent, pos):
        self.pos = pos
        self.prev = parent

# Astar planning
def astar(start, goal, distMethod, expandMethod, robot, env):

    # Init
    # print 'Planning Query: from %s to %s'%(str(start),str(goal))
    prevKey = getKey(start)
    h = { prevKey:heuristic(start, goal, distMethod) }
    g = { prevKey:0 }
    frontiers = [ (h[prevKey]*H_PRIOR+g[prevKey], Node(None, np.array(start))) ]
    goalNode = None; explored = []; collided = [];
    it = 0

    # Astar main loop
    while len(frontiers)!=0 and it<MAX_ITER:

        # Goal Test
        # percent = float(100*it)/MAX_ITER
        # print '{0}\r'.format('|' * int(percent+1) + '%s'%str(percent) + '%'),
        node = hq.heappop(frontiers)[1]
        prevKey = getKey(node.pos)
        explored.append(node.pos)
        if goaltest(node.pos, goal, distMethod):
            goalNode = node
            break

        # Expand successor
        for point in expandMethod(node.pos):
            key = getKey(point)
            step_cost = heuristic(point, node.pos, distMethod)
            if (key in g) and (g[key]<=g[prevKey]+step_cost):
                continue
            if isCollided(point, robot, env):
                collided.append(point)
                continue
            h[key] = heuristic(point, goal, distMethod)
            g[key] = g[prevKey] + step_cost
            hq.heappush(frontiers, (h[key]*H_PRIOR+g[key], Node(node, point)))
        it += 1
    #try: 
        # print '\r{0}'.format(' '*120) + '\rOptimization: %s'%(str( g[prevKey]))
        # print 'Iteration: ' + '|' * int(percent+1) + '%s'%str(percent) + '%'
    #except:
        #pass
    #finally:
    return goalNode, explored, collided

# Anastar cost
def getCost(G, g, h):
    return (G-g)/h

# Goal test
def goaltest(pos, goal, distMethod):
    return heuristic(pos, goal, distMethod) < MAX_ERR

# Heuristic loss
def heuristic(pos1, pos2, distMethod):
    return distMethod(pos1, pos2)

# Collision test
def isCollided(pos, robot, env):
    robot.SetActiveDOFValues(pos)
    for i,body in enumerate(env.GetBodies()):
        if env.CheckCollision(robot, body):
            return True
    return False
