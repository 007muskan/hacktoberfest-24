import heapq

class Node:
    def __init__(self, name, parent=None, g=0, h=0):
        self.name = name  # Node name (e.g., a position)
        self.parent = parent  # Parent node
        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic cost to goal
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f  # Less than for priority queue

def heuristic(node, goal):
    # Example heuristic: Euclidean distance
    return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

def a_star(start, goal, graph):
    open_set = []  # Nodes to be evaluated
    closed_set = set()  # Nodes already evaluated
    start_node = Node(start, None, 0, heuristic(start, goal))
    
    heapq.heappush(open_set, start_node)  # Push the start node

    while open_set:
        current_node = heapq.heappop(open_set)  # Get node with lowest f
        
        # If we reach the goal, reconstruct the path
        if current_node.name == goal:
            path = []
            while current_node:
                path.append(current_node.name)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path
        
        closed_set.add(current_node.name)  # Mark node as evaluated

        # Evaluate neighbors
        for neighbor, cost in graph[current_node.name].items():
            if neighbor in closed_set:
                continue  # Ignore already evaluated neighbors
            
            g_cost = current_node.g + cost  # Cost to reach the neighbor
            h_cost = heuristic(neighbor, goal)  # Heuristic cost to goal
            neighbor_node = Node(neighbor, current_node, g_cost, h_cost)

            # Check if neighbor is already in open set
            if any(neighbor_node.name == node.name and g_cost >= node.g for node in open_set):
                continue  # Ignore if it's not a better path
            
            heapq.heappush(open_set, neighbor_node)  # Add neighbor to open set

    return None  # No path found

# Example graph as an adjacency list
graph = {
    (0, 0): {(0, 1): 1, (1, 0): 1.5},
    (0, 1): {(0, 0): 1, (0, 2): 1, (1, 1): 1.5},
    (0, 2): {(0, 1): 1, (1, 2): 1.5},
    (1, 0): {(1, 1): 1, (0, 0): 1.5},
    (1, 1): {(1, 0): 1, (1, 2): 1, (0, 1): 1.5},
    (1, 2): {(1, 1): 1, (0, 2): 1.5},
}

# Example usage
start = (0, 0)
goal = (1, 2)
path = a_star(start, goal, graph)

print("Path from start to goal:", path)
