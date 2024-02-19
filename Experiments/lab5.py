#Eight solution game puzzle by A* algorithm
#9521_RisaAlmeida
import heapq
import numpy as np

class Node:
    def __init__(self, state, parent=None, g=0, h=0):
        self.state = state
        self.parent = parent
        self.g = g  # Cost from the start node to the current node
        self.h = h  # Heuristic cost from the current node to the goal node

    def __lt__(self, other):
        # Comparison method for priority queue
        return (self.g + self.h) < (other.g + other.h)

def manhattan_distance(state, goal):
    # Calculate Manhattan Distance heuristic
    distance = 0
    for i in range(3):
        for j in range(3):
            if state[i, j] != 0:
                goal_position = np.where(goal == state[i, j])
                distance += abs(i - goal_position[0]) + abs(j - goal_position[1])
    return distance

def get_successors(node):
    # Generate successor nodes by moving tiles
    successors = []
    zero_position = np.where(node.state == 0)

    for move in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        new_position = (zero_position[0] + move[0], zero_position[1] + move[1])

        if 0 <= new_position[0] < 3 and 0 <= new_position[1] < 3:
            new_state = np.copy(node.state)
            new_state[zero_position], new_state[new_position] = new_state[new_position], new_state[zero_position]
            successors.append(Node(new_state, node))

    return successors

def a_star(initial_state, goal_state):
    open_list = [Node(initial_state, None, 0, manhattan_distance(initial_state, goal_state))]
    closed_set = set()

    while open_list:
        current_node = heapq.heappop(open_list)

        if np.array_equal(current_node.state, goal_state):
            # Reconstruct the solution path
            solution_path = []
            while current_node:
                solution_path.append(current_node.state)
                current_node = current_node.parent
            return solution_path[::-1]

        closed_set.add(tuple(current_node.state.flatten()))

        for successor in get_successors(current_node):
            if tuple(successor.state.flatten()) not in closed_set:
                successor.g = current_node.g + 1
                successor.h = manhattan_distance(successor.state, goal_state)
                heapq.heappush(open_list, successor)

    return None

def print_solution_path(solution_path):
    for i, state in enumerate(solution_path):
        print(f"Step {i + 1}:")
        print(state)
        print()

if __name__ == "__main__":
    initial_state = np.array([[1, 2, 3], [4, 0, 5], [6, 7, 8]])
    goal_state = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]])

    solution_path = a_star(initial_state, goal_state)

    if solution_path:
        print("Solution Path:")
        print_solution_path(solution_path)
    else:
        print("No solution found.")

#output:
#Solution Path:
#Step 1:
#[[1 2 3]
 #[4 0 5]
# [6 7 8]]

#Step 2:
#[[1 2 3]
 #[0 4 5]
 #[6 7 8]]

#Step 3:
#[[0 2 3]
 #[1 4 5]
 #[6 7 8]]

#Step 4:
#[[2 0 3]
 #[1 4 5]
# [6 7 8]]

#Step 5:
#[[2 3 0]
 #[1 4 5]
 #[6 7 8]]

#Step 6:
#[[2 3 5]
 #[1 4 0]
 #[6 7 8]]

#Step 7:
#[[2 3 5]
 #[1 0 4]
# [6 7 8]]

#Step 8:
#[[2 0 5]
 #[1 3 4]
 #[6 7 8]]

#Step 9:
#[[0 2 5]
 #[1 3 4]
# [6 7 8]]

#Step 10:
#[[1 2 5]
 #[0 3 4]
 #[6 7 8]]

#Step 11:
#[[1 2 5]
 #[3 0 4]
 #[6 7 8]]

#Step 12:
#[[1 2 5]
 #[3 4 0]
 #[6 7 8]]

#Step 13:
#[[1 2 0]
 #[3 4 5]
 #[6 7 8]]

#Step 14:
#[[1 0 2]
 #[3 4 5]
 #[6 7 8]]

#Step 15:
#[[0 1 2]
# [3 4 5]
# [6 7 8]]


