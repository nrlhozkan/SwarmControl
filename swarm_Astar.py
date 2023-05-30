import numpy as np
import matplotlib.pyplot as plt
from heapq import heappop, heappush

# Constants
GRID_SIZE = 10
OBSTACLES = [(3, 4), (7, 5)]
GOAL = (8, 2)
START = [(1, 1), (1, 3), (1, 5), (1, 7)]
PID_PARAMETERS = {'Kp': .435, 'Ki': 0.01, 'Kd': .41}
DT = 0.1

# A* Path Planning
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def astar(start, goal):
    open_list = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    
    while open_list:
        current = heappop(open_list)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            new_g_score = g_score[current] + 1
            
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and neighbor not in OBSTACLES:
                if neighbor not in g_score or new_g_score < g_score[neighbor]:
                    g_score[neighbor] = new_g_score
                    priority = new_g_score + heuristic(neighbor, goal)
                    heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current
        
# PID Controller
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.error_sum = 0
        self.prev_error = 0
    
    def calculate_control(self, error):
        self.error_sum += error * self.dt
        error_diff = (error - self.prev_error) / self.dt
        control = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_diff
        self.prev_error = error
        return control

# Consensus Formation
def consensus_formation(current_positions, goal):
    x_sum = goal[0]
    y_sum = goal[1]
    num_neighbors = 1
    
    for position in current_positions:
        x_sum += position[0]
        y_sum += position[1]
        num_neighbors += 1
    
    avg_x = x_sum / num_neighbors
    avg_y = y_sum / num_neighbors
    
    desired_positions = []
    for i in range(num_neighbors - 1):
        angle = i * (2 * np.pi / (num_neighbors - 1))
        x = avg_x + np.cos(angle)
        y = avg_y + np.sin(angle)
        desired_positions.append((x, y))
    
    return desired_positions

# Collision Avoidance
def collision_avoidance(current_positions, desired_positions):
    adjusted_positions = []
    
    for i in range(len(current_positions)):
        current_x, current_y = current_positions[i]
        desired_x, desired_y = desired_positions[i]
        
        # Check for collision with obstacles
        if any((current_x, current_y) == obs for obs in OBSTACLES):
            adjusted_positions.append(current_positions[i])
            continue
        
        # Check for collision with other UAVs
        collision = False
        for j in range(len(current_positions)):
            if i != j:
                other_x, other_y = current_positions[j]
                distance = np.sqrt((current_x - other_x) ** 2 + (current_y - other_y) ** 2)
                if distance < 0.25:
                    collision = True
                    print("collision!")
                    break
        
        if collision:
            adjusted_positions.append(current_positions[i])
        else:
            adjusted_positions.append(desired_positions[i])
    
    return adjusted_positions

# Main Function
def main():
    # Path Planning
    paths = []
    index=1
    for start in START:
        path = astar(start, GOAL)
        paths.append(path)
        my_string = "UAV"+str(index)+" = "+str(path)
        index+=1
        print(my_string)
    
    # Initialize UAV Positions
    current_positions = START.copy()
    swarm_positions = [current_positions.copy()]
    
    # Initialize PID Controllers
    controllers = []
    for _ in range(4):
        controller = PIDController(PID_PARAMETERS['Kp'], PID_PARAMETERS['Ki'], PID_PARAMETERS['Kd'], DT)
        controllers.append(controller)
    
    # Simulation
    timesteps = len(paths[0])
    for t in range(1, timesteps):
        # Formation Control
        desired_positions = consensus_formation(current_positions, GOAL)
        
        # Collision Avoidance
        adjusted_positions = collision_avoidance(current_positions, desired_positions)
        
        # UAV Movement
        for i in range(4):
            x, y = current_positions[i]
            desired_x, desired_y = adjusted_positions[i]
            
            # PID Trajectory Tracking
            error_x = desired_x - x
            error_y = desired_y - y
            control_x = controllers[i].calculate_control(error_x)
            control_y = controllers[i].calculate_control(error_y)
            
            # Update UAV Position
            new_x = x + control_x * DT
            new_y = y + control_y * DT
            current_positions[i] = (new_x, new_y)
        
        swarm_positions.append(current_positions.copy())
        
        # Check if all UAVs have reached the goal
        if all(np.linalg.norm(np.array(pos) - np.array(GOAL)) < 0.5 for pos in current_positions):
            break
    
    # Plotting
    for i in range(4):
        x = [pos[i][0] for pos in swarm_positions]
        y = [pos[i][1] for pos in swarm_positions]
        plt.plot(x, y, label='UAV {}'.format(i+1))
    
    plt.scatter([GOAL[0]], [GOAL[1]], color='r', marker='x', label='Goal')
    plt.scatter([obs[0] for obs in OBSTACLES], [obs[1] for obs in OBSTACLES], color='k', marker='s', label='Obstacle')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Swarm Movement')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
