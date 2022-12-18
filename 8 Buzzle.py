import argparse
import timeit
import resource
from collections import deque
from heapq import heappush, heappop, heapify
import itertools

################################################## Global Variables ##################################################
goal_state = [0, 1, 2, 3, 4, 5, 6, 7, 8]
goal_node = State
initial_state = list()
board_len = 0
board_side = 0
nodes_expanded = 0
max_search_depth = 0
max_frontier_size = 0
moves = list()
costs = set()

################################################## Class Node ##################################################
class Node:
    def __init__(self,state,parent,action,depth,cost,key):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.depth = depth
        self.key = key

        if self.state:
            self.map = ''.join(str(e) for e in self.state)

    def __eq__(self, other):
        return self.map == other.map

    def __lt__(self, other):
        return self.map < other.map
    
################################################# Class EightBuzzle #################################################        
################################################## GoalTest Method ##################################################       
class EightBuzzle:
    def GoalTest(self,state):
        return self.state == goal_state

################################################## DepthFirstSearch Method ##################################################     
    def DFS(initial_state):
        global max_fringe_size, goal_node, max_search_depth
        explored, stack = set(), list([Node(initial_state, None, None, 0, 0, 0)])
        while stack:
              node = stack.pop()
              explored.add(node.map)
              if node.state == goal_state:
                  goal_node = node
                  return stack
              neighbors = reversed(Expand(node))
              for neighbor in neighbors:
                 if neighbor.map not in explored:
                     stack.append(neighbor)
                     explored.add(neighbor.map)
                 if neighbor.depth > max_search_depth:
                     max_search_depth += 1
                 if len(stack) > max_fringe_size:
                     max_fringe_size = len(stack)

################################################## BreadthFirstSearch Method ##################################################                 
    def BFS(initial_state):
        global max_fringe_size, goal_node, max_search_depth
        explored, queue = set(), deque([Node(initial_state, None, None, 0, 0, 0)])
        while queue:
              node = queue.popleft()
              explored.add(node.map)
              if node.state == goal_state:
                  goal_node = node
                  return queue
              neighbors = Expand(node)
              for neighbor in neighbors:
                   if neighbor.map not in explored:
                       queue.append(neighbor)
                       explored.add(neighbor.map)
                   if neighbor.depth > max_search_depth:
                       max_search_depth += 1
                   if len(queue) > max_fringe_size:
                       max_fringe_size = len(queue)

################################################## A* Search Method ##################################################                 
     def AStar(initial_state):
         global max_fringe_size, goal_node, max_search_depth
         explored, heap, heap_entry, counter = set(), list(), {}, itertools.count()
         key = H(inital_state)
         root = Node(inital_state, None, None, 0, 0, key)
         entry = (key, 0, root)
         heappush(heap, entry)
         heap_entry[root.map] = entry
         while heap:
               node = heappop(heap)
               explored.add(node[2].map)
               if node[2].state == goal_state:
                   goal_node = node[2]
                   return heap
               neighbors = Expand(node[2])
               for neighbor in neighbors:
                    neighbor.key = neighbor.cost + H(neighbor.state)
                    entry = (neighbor.key, neighbor.move, neighbor)
                    if neighbor.map not in explored:
                        heappush(heap, entry)
                        explored.add(neighbor.map)
                        heap_entry[neighbor.map] = entry
                    if neighbor.depth > max_search_depth:
                        max_search_depth += 1
                    elif neighbor.map in heap_entry and neighbor.key < heap_entry[neighbor.map][2].key:
                          hindex = heap.index((heap_entry[neighbor.map][2].key,
                                   heap_entry[neighbor.map][2].move,
                                   heap_entry[neighbor.map][2]))
                          heap[int(hindex)] = entry
                          heap_entry[neighbor.map] = entry
                          heapify(heap)
              if len(heap) > max_fringe_size:
                   max_fringe_size = len(heap)

################################################## Heuristic Method ##################################################               
    def H(state):
        return sum(abs(b % board_side - g % board_side) + abs(b//board_side - g//board_side)
                   for b, g in ((state.index(i), goal_state.index(i)) for i in range(1, board_len)))

################################################## Move Method ##################################################
    def Move(state,position):
         new_state = state[:]
         index = new_state.index(0)
         if position == 1:  # Up
             if index not in range(0, board_side):
                 temp = new_state[index - board_side]
                 new_state[index - board_side] = new_state[index]
                 new_state[index] = temp
                 return new_state
             else:
                 return None
         if position == 2:  # Down
             if index not in range(board_len - board_side, board_len):
                 temp = new_state[index + board_side]
                 new_state[index + board_side] = new_state[index]
                 new_state[index] = temp
                 return new_state
             else:
                 return None
         if position == 3:  # Left
             if index not in range(0, board_len, board_side):
                 temp = new_state[index - 1]
                 new_state[index - 1] = new_state[index]
                 new_state[index] = temp
                 return new_state
             else:
                 return None
         if position == 4:  # Right
             if index not in range(board_side - 1, board_len, board_side):
                 temp = new_state[index + 1]
                 new_state[index + 1] = new_state[index]
                 new_state[index] = temp
                 return new_state
             else:
                 return None

################################################## BackTrace Method ##################################################          
    def BackTrace():
         current_node = goal_node
         while initial_state != current_node.state:
             if current_node.move == 1:
                 movement = 'Up'
             elif current_node.move == 2:
                 movement = 'Down'
             elif current_node.move == 3:
                 movement = 'Left'
             else:
                 movement = 'Right'
             moves.insert(0, movement)
             current_node = current_node.parent
         return moves

################################################## Export Method ##################################################     
    def Export(fringe,time):
         global moves
         moves = BackTrace()
         print("\ncost_of_path: " + str(len(moves)))
         print("\nnodes_expanded: " + str(nodes_expanded))
         print("\nfringe_size: " + str(len(fringe)))
         print("\nmax_fringe_size: " + str(max_fringe_size))
         print("\nsearch_depth: " + str(goal_node.depth))
         print("\nmax_search_depth: " + str(max_search_depth))
         print("\nrunning_time: " + format(time, '.8f'))

################################################## Expand Method ##################################################     
    def Expand(node):
        global nodes_expanded
        nodes_expanded += 1
        neighbors = list()
        neighbors.append(Node(Move(node.state, 1), node, 1, node.depth + 1, node.cost + 1, 0))
        neighbors.append(Node(Move(node.state, 2), node, 2, node.depth + 1, node.cost + 1, 0))
        neighbors.append(Node(Move(node.state, 3), node, 3, node.depth + 1, node.cost + 1, 0))
        neighbors.append(Node(Move(node.state, 4), node, 4, node.depth + 1, node.cost + 1, 0))
        nodes = [neighbor for neighbor in neighbors if neighbor.state]
        return nodes

################################################## Read Method ################################################## 
    def Read(configuration):
        global board_len, board_side
        data = configuration.split(",")
        for element in data:
            initial_state.append(int(element))
        board_len = len(initial_state)
        board_side = int(board_len ** 0.5)
       
################################################## Main ##################################################
def main():

    parser = argparse.ArgumentParser()

    parser.add_argument('algorithm')
    parser.add_argument('board')
    args = parser.parse_args()

    Read(args.board)

    function = function_map[args.algorithm]

    start = timeit.default_timer()

    fringe = function(initial_state)

    stop = timeit.default_timer()

    Export(fringe, stop-start)


function_map = {
    'BFS': BFS,
    'DFS': DFS,
    'AStar': AStar,
}

if __name__ == '__main__':
    main()
