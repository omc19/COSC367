import math
from math import *
from search import *
from heapq import *




class RoutingGraph(Graph):
    """ """
    def __init__(self, map_str):
        self.map_str = map_str
        
        self.mapy = self.create_map(map_str)
        self.type_dict = {"Agent" : "S0123456789",
                     "Obstacle": 'X',
                     "Wall": '+-|',
                     "Goal": 'G',
                     "Fuel": 'F',
                     "Start" : 'S'
                     }
        
        self.directions = [('N' , -1, 0),
                           ('NE', -1, 1),
                           ('E' ,  0, 1),
                           ('SE',  1, 1),
                           ('S' ,  1, 0),
                           ('SW',  1, -1),
                           ('W' ,  0, -1),
                           ('NW', -1, -1)]
        self.goal = self.goal_node() # coordinate of goal node
          
    
    def goal_node(self):
        """"""
        goal = []
        for row in range(0,len(self.mapy)):
            for col in range(0,len(self.mapy[row])):
                if self.mapy[row][col] in self.type_dict['Goal']:
                    goal.append((row, col))
                
        return goal
    
    def is_goal(self, node):
        """Returns true if the given node is a goal state, false otherwise."""
        row, col, fuel = node
        return self.mapy[row][col] in self.type_dict['Goal']
            

    def starting_nodes(self):
        """Returns a sequence of starting nodes. Often there is only one
        starting node but even then the function returns a sequence
        with one element. It can be implemented as an iterator if
        needed.

        """
        starting = []
        for row in range(0,len(self.mapy)):
            for col in range(0,len(self.mapy[row])):
                if self.mapy[row][col] in self.type_dict['Agent'] and self.mapy[row][col] == "S" :
                    starting.append((row, col, math.inf))
                elif self.mapy[row][col] in self.type_dict['Agent']:
                    starting.append((row, col, int(self.mapy[row][col])))
        return starting


    def outgoing_arcs(self, tail_node):
        """Given a node it returns a sequence of arcs (Arc objects)
        which correspond to the actions that can be taken in that
        state (node)."""
        moveable = []
        row, column, fuel = tail_node
        for direct in self.directions:
            label = direct[0]
            rows = direct[1] + row
            cols = direct[2] + column
            fuel_level = fuel - 1 
            head_set = (rows, cols, fuel_level)
            if self.mapy[rows][cols] not in self.type_dict['Obstacle'] and self.mapy[rows][cols] not in self.type_dict['Wall']:
                if fuel_level >= 0:
                    yield Arc(tail_node, head_set, label, 2)
                        
        if self.mapy[row][column] in self.type_dict['Fuel']:
            if fuel != 9:
                yield Arc(tail_node, (row, column, 9), 'Fuel up', 5)
           
    
    def manhattan_distance(self, point_1, point_2):
        """ """       
        
        moving_cost = 2
        x1, y1, fuel = point_1
        x2, y2 = point_2
        man_x = (abs(x1-x2)) 
        man_y = (abs(y1-y2)) 
        return moving_cost * max(man_x, man_y) #- moving_cost * min(man_x, man_y) #diagonal distance with a uniform cost
               
        
     
            
    def estimated_cost_to_goal(self, node):
        """Return the estimated cost to a goal node from the given
        state. This function is usually implemented when there is a
        single goal state. The function is used as a heuristic in
        search. The implementation should make sure that the heuristic
        meets the required criteria for heuristics."""
        
        cos  = self.manhattan_distance(node, self.goal[0])

        return cos
    
    def create_map(self, map_str):
        """ """
        map_list = []
        for i in map_str.split('\n'):
            i = i.strip()
            map_list.append(i)
        
        return map_list


class AStarFrontier(Frontier):
    """ """

    def __init__(self, map_str):
        self.expanded = []
        self.visited = set()
        self.p_queue = []
        self.map_str = map_str
        
    
    def add(self, path):
        label = path[-1].label
        row,col,fuel = path[-1].head      
        #x_y = (row,col)    
        if (row,col,fuel) in self.visited and label != "Fuel up":
            return
        cost = 0
        for i in path:
            cost += i[3]
        cost = cost + self.map_str.estimated_cost_to_goal((row,col,fuel)) #should be estimated cost but not needed right now
        #print(cost)
        #print(path)
        self.p_queue.append((cost, path))
    
    def __iter__(self):
        while len(self.p_queue) != 0:
            other_path = []           
            lowest_cost = self.p_queue[0] #picking the fisrt element of the list (lowest cost)
            for cost, path in self.p_queue:
                #print(lowest_cost[0])
                cost_low = lowest_cost[0]
                if cost < cost_low:
                    other_path = []
                    other_path.append((cost, path))
                    #print(cost, path)                    
                    lowest_cost = (cost, path)
                elif cost == cost_low:
                    other_path.append((cost, path))
                    #print(cost, path)
            #print(lowest_cost)    
            #label = path[-1].label 
            #if label == "Fuel up":
                #self.visited = set() #clear the set so it can trace back the way it came
 
            for i in other_path:
                row,col,fuel = i[1][-1].head
                self.p_queue.remove(i)
                #print(self.visited)
                #print(self.p_queue)
                if (row,col,fuel) not in self.visited:
                    self.visited.add((row,col,fuel)) #add the head to the visited set
                    yield i[1]            


def print_map(map_graph, frontier, solution):
    """ """
    new_map = []
    for i in map_graph.mapy:
        i = i.strip()
        new_map.append(list(i))
    
    #for i in
    #print(frontier.visited)
    
    for coord in frontier.visited:
        row = coord[0]
        col = coord[1]
        if new_map[row][col] not in map_graph.type_dict['Start'] and new_map[row][col] not in map_graph.type_dict['Goal']:
            new_map[row][col] = '.'
    if solution:
        for arc in solution:
            if arc[0]:
                tail = arc[0]
                coord = (tail[0], tail[1])
                row = coord[0]
                col = coord[1]
                x_y = new_map[row][col]
                if x_y not in map_graph.type_dict['Start'] and new_map[row][col] not in map_graph.type_dict['Goal']:
                    new_map[row][col] = "*"    
    printable_map = ""
    for i in new_map:
        p_map = "".join(i)
        printable_map += p_map + '\n'
    print(printable_map)    
    
    
        
    # how to print the map
    #printable_map = ""
    #for i in map_graph.mapy:
        #printable_map += i+'\n'
    #print(printable_map)
    
    
    #print(map_graph.mapy)
    #print(len(frontier.expanded))
    #print(frontier.expanded)
    #for i in frontier.expanded:
        #print(i)
    #print('\n')
    #print(solution)
    #print(solution[0].head)
            

    #def __init__(self, map_str):
        #self.expanded = []
        #self.visited = set()
        #self.p_queue = []
        #self.map_str = map_str
        ##self.priority = 0 # this is for if a path has the same cost as another
                          ## use this variable to distinguish priority    
    
    #def add(self, path):
        #label = path[-1].label
        #row,col,fuel = path[-1].head      
        ##x_y = (row,col)    
        #if (row,col,fuel) in self.visited and label != "Fuel up":
            
            #return
        #cost = path[-1].cost + self.map_str.estimated_cost_to_goal((row,col,fuel)) #should be estimated cost but not needed right now
        ##print(cost)
        ##print(path)
        ##self.expanded.append(path)
        #self.p_queue.append((cost, path))
    
    #def __iter__(self):
        #while len(self.p_queue) != 0:
            #other_path = []
            
            ##tail,head,label,cost = self.p_queue[0][2][0]
            ##print(self.p_queue)
            ##prior, cost, path = heappop(self.p_queue)    
            
            #lowest_cost = self.p_queue[0] #picking the fisrt element of the list (lowest cost)
            #for cost, path in self.p_queue:
                ##print(lowest_cost[0])
                #cost_low = lowest_cost[0]
                #if cost <= cost_low:
                    #other_path.append((cost, path))
                    #print(cost, path)                    
                    #lowest_cost = (cost, path)
                ##elif cost == cost_low:
                    ##other_path.append((cost, path))
                    ##print(cost, path)
            ##print(lowest_cost)    
            ##label = path[-1].label 
            ##if label == "Fuel up":
                ##self.visited = set() #clear the set so it can trace back the way it came
                
            #print(other_path, 'yeet')   
            #for i in other_path:
                #print(i)
                #print(i[1][-1].head)
            #row,col,fuel = lowest_cost[1][-1].head
            #print(row,col,fuel ,'hm')
            
            #self.p_queue.remove(lowest_cost)
            ##print(self.visited)
            ##print(self.p_queue)
            #if (row,col,fuel) not in self.visited:
                #self.visited.add((row,col,fuel)) #add the head to the visited set
                #yield lowest_cost[1]  

### ------------------ wrong but right first attempt--------------------------------

    #def __init__(self, map_str):
        #self.visited = set()
        #self.p_queue = []
        #self.map_str = map_str
        #self.priority = 0 # this is for if a path has the same cost as another
                          ## use this variable to distinguish priority
               
    
    #def add(self, path):
        #cost = 0 #cost of entire path
        #for i in path:
            #cost += i[3] #Add all the cost of the paths together so can pull out the path accordingly
        #label = path[-1].label
        #row,col,fuel = path[-1].head      
        #x_y = (row,col)
        #discovered = self. priority, cost, path
        #if x_y in self.visited and label != 'Fuel up': #if the path isn't in the visited set
            #return
        #heappush(self.p_queue, discovered)            # or on a fuel push it to the heap
        #self.priority += 1


    #def __iter__(self):
        #while len(self.p_queue) != 0:
            ##tail,head,label,cost = self.p_queue[0][2][0]
            ##print(self.p_queue)
            #prior, cost, path = heappop(self.p_queue)
            #row,col,fuel = path[-1].head        
            #x_y = (row,col)
            #label = path[-1].label
            ##if prior = 250:
                ##yeild Arc(tail=(4, 14, 7), head=(2, 6, 6), label='SE', cost=2)
            #if label == 'Fuel up':
                #print("clear visited")  
                #self.visited = set()  #clear the set so it can trace back the way it came      
                #print(self.visited)
            #if x_y in self.visited:
                #continue
            #else:
                #print("chose this path "+str(path[-1])) 
                #self.visited.add(x_y) #add the head to the visited set
                #yield path

### -------------------- Theses has beens ----------------------------------###

    #def __iter__(self):
        #while len(self.p_queue) != 0:
            #tail,head,label,cost = self.p_queue[0][2][0]
            #print(self.p_queue)
            #prior, cost, path = heappop(self.p_queue)
            #row,col,fuel = path[-1].head        
            #x_y = (row,col)
            #label = path[-1].label
            #if x_y not in self.visited or label == 'Fuel up':
                #if label == 'Fuel up':
                    #self.visited = set() #clear the set so it can trace back the way it came
                #else:
                    #self.visited.add(x_y) #add the head to the visited set
                    #yield path
                    
    #def add(self, path):
        #cost = 0 #cost of entire path
        #for i in path:
            #cost += i[3] #Add all the cost of the paths together so can pull out the path accordingly
        #label = path[-1].label
        #row,col,fuel = path[-1].head      
        #x_y = (row,col)
        #discovered = self. priority, cost, path
        #if x_y not in self.visited or label == 'Fuel up': #if the path isn't in the visited set
            #heappush(self.p_queue, discovered)            # or on a fuel push it to the heap
            #self.priority += 1


    #def __iter__(self):
        #while len(self.p_queue) != 0:
            #tail,head,label,cost = self.p_queue[0][2][0]
            ##print(self.p_queue)
            #prior, cost, path = heappop(self.p_queue)
            #row,col,fuel = path[-1].head        
            #x_y = (row,col)
            #label = path[-1].label
            #if label == 'Fuel up':
                #self.visited = set()  #clear the set so it can trace back the way it came          
            #if x_y not in self.visited:
                #self.visited.add(x_y) #add the head to the visited set
                #yield path                    
                    



def main():




    map_str = """\
    +-------------+
    |         X   |
    | S       X G |
    |         X   |
    +-------------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)

    




if __name__ == "__main__":
    main()
    



    #map_str = """\
    #+---------+
    #|    X    |
    #| S  X   G|
    #|    X    |
    #+---------+
    #"""    


    #map_str = """\
        #+---------+
        #|2        |
        #|    G 123|
        #|2XXXXXXX |
        #|  F      |
        #+---------+
        #"""
        
        #21
        
        #map_str = """\
        #+----------------+
        #|2             F |
        #|XX   G 123      |
        #|2XXXXXXXXXXXXXX |
        #|  F             |
        #|           F    |
        #+----------------+
        #"""   
        
        #Actions:
          #SE,
          #E,
          #Fuel up,
          #E,
          #E,
          #E,
          #E,
          #E,
          #E,
          #E,
          #E,
          #SE,
          #Fuel up,
          #NE,
          #E,
          #E,
          #NE,
          #N,
          #NW,
          #Fuel up,
          #SW,
          #W,
          #W,
          #W,
          #W,
          #W,
          #W,
          #W,
          #W.
        #Total cost: 67        