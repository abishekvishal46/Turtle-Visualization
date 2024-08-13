import turtle
import heapq

# Global variables
nodes = {
    "3": (300, 200),
    "1": (100, 200),
    "2": (100, 100),
    "4": (300, 100),
    "5": (100, 0),
    "6": (300, 0),
    "7": (100, -100),
    "8": (300, -100),
    "9": (-100, -100),
    "10": (-100, 0),
    "11": (-100, 100),
    "12": (-400, 0),
    "13": (-400, -100),
    "14": (-300, -100),
    "15": (-400, 100),
    "16": (-300, -50)
}

edges = {
    "1": ["2", "3"],
    "2": ["1", "4", "5", "11"],
    "3": ["1", "4"],
    "4": ["3", "2", "6"],
    "5": ["2", "6", "7", "10"],
    "6": ["4", "5", "8"],
    "7": ["5", "8", "9"],
    "8": ["6", "7"],
    "9": ["7", "10", "14"],
    "10": ["9", "12", "11", "5"],
    "11": ["10", "2"],
    "14": ["9", "13", "16"],
    "13": ["14", "12"],
    "12": ["13", "15", "10"],
    "15":["12"],
    "16": ["14"]
}

obstacles = set()
path_index = 0
moving = True
path = []
start_node = ""
goal_node = ""
step_size = 5  # Distance to move in each step

# Function to draw the map
def draw_map():
    drawer = turtle.Turtle()
    drawer.speed("fastest")
    drawer.penup()
    for node, position in nodes.items():
        drawer.goto(position)
        drawer.dot(20, "blue")
        drawer.write(node, align="center", font=("Arial", 12, "normal"))

    drawer.pencolor("black")
    drawer.pensize(2)
    for node, neighbors in edges.items():
        start_pos = nodes[node]
        for neighbor in neighbors:
            end_pos = nodes[neighbor]
            drawer.goto(start_pos)
            drawer.pendown()
            drawer.goto(end_pos)
            drawer.penup()

    drawer.hideturtle()

# Heuristic function for A* (Manhattan distance)
def heuristic(node1, node2):
    x1, y1 = nodes[node1]
    x2, y2 = nodes[node2]
    return abs(x1 - x2) + abs(y1 - y2)

# A* pathfinding algorithm
def a_star(start, goal, avoid_edge=None):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float("inf") for node in nodes}
    g_score[start] = 0
    f_score = {node: float("inf") for node in nodes}
    f_score[start] = heuristic(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in edges[current]:
            if (current, neighbor) == avoid_edge or (neighbor, current) == avoid_edge:
                continue
            if neighbor in obstacles:
                continue
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Function to move incrementally towards a target position
def move_towards_target():
    global path_index, moving, path

    if path_index < len(path) and moving:
        current_node = path[path_index]
        target_position = nodes[current_node]
        current_position = agv.position()

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        if distance <= step_size:
            agv.goto(target_position)
            path_index += 1
            if current_node == goal_node:
                agv.dot(10, "green")
                return
        else:
            angle = agv.towards(target_position)
            agv.setheading(angle)
            agv.forward(step_size)

        screen.ontimer(move_towards_target, 10)

# Function to move back to the previous node and stop
def backtrack():
    global path_index, moving, path

    if path_index > 0:
        previous_node = path[path_index - 1]
        target_position = nodes[previous_node]
        current_position = agv.position()

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        if distance <= step_size:
            agv.goto(target_position)
            path_index -= 1
            # Recalculate path from the new position, avoiding the specific edge
            new_start = path[path_index]
            avoid_edge = (path[path_index], path[path_index + 1]) if path_index + 1 < len(path) else None
            new_path = a_star(new_start, goal_node, avoid_edge=avoid_edge)
            if new_path and new_path != path:
                path = new_path
                path_index = 0
                start_movement()
            else:
                print("No alternative path found")
                moving = False
                return
        else:
            angle = agv.towards(target_position)
            agv.setheading(angle)
            agv.forward(step_size)
            screen.ontimer(backtrack, 10)
    else:
        moving = False

def stop_movement():
    global moving
    moving = False
    backtrack()

def start_movement():
    global moving
    moving = True
    move_towards_target()

# Main function to setup and run the AGV Path Tracker
def main():
    global screen, agv, start_node, goal_node, path

    screen = turtle.Screen()
    screen.title("AGV Path Tracker")
    screen.setup(width=800, height=600)

    agv = turtle.Turtle()
    agv.shape("turtle")
    agv.speed("slowest")
    agv.penup()

    draw_map()

    start_node = screen.textinput("Input", "Enter the start node:")
    goal_node = screen.textinput("Input", "Enter the goal node:")

    path = a_star(start_node, goal_node)

    if path:
        move_towards_target()
    else:
        print("No path found")

    screen.listen()
    screen.onkey(stop_movement, "q")
    screen.onkey(start_movement, "w")
    screen.mainloop()

if __name__ == "__main__":
    main()