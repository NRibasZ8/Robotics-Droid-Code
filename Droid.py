from machine import Pin, PWM, I2C, Timer
import time
import math
from collections import defaultdict, deque
import heapq

# ==================== HARDWARE CONFIGURATION ====================
# Line Sensor Pins
S1 = Pin(13, Pin.IN)  # Far left sensor
S2 = Pin(12, Pin.IN)  # Middle left sensor
S3 = Pin(14, Pin.IN)  # Center sensor
S4 = Pin(27, Pin.IN)  # Middle right sensor
S5 = Pin(26, Pin.IN)  # Far right sensor

# Limit Switch and Proximity Sensor
CLP = Pin(25, Pin.IN, Pin.PULL_UP)    # Limit switch (with pullup)
NEAR = Pin(33, Pin.IN)   # Proximity sensor

# Motor Control Pins
LEFT_MOTOR_PWM = PWM(Pin(32))
LEFT_MOTOR_DIR = Pin(23, Pin.OUT)
RIGHT_MOTOR_PWM = PWM(Pin(18))
RIGHT_MOTOR_DIR = Pin(19, Pin.OUT)

# Electromagnet Control Pins (one per face)
EMAG1 = Pin(15, Pin.OUT)  # Face 1 electromagnet
EMAG2 = Pin(2, Pin.OUT)   # Face 2 electromagnet
EMAG3 = Pin(4, Pin.OUT)   # Face 3 electromagnet
EMAG4 = Pin(5, Pin.OUT)   # Face 4 electromagnet

# Mechano Wheel Control
MECHANO_ENGAGE = Pin(17, Pin.OUT)  # HIGH = engaged, LOW = disengaged

# TOF Sensor I2C Addresses
TOF1_ADDRESS = 0x30  # Face 1 TOF
TOF2_ADDRESS = 0x31  # Face 2 TOF
TOF3_ADDRESS = 0x32  # Face 3 TOF
TOF4_ADDRESS = 0x33  # Face 4 TOF

# ==================== NAVIGATION CONSTANTS ====================
BASE_SPEED = 0.6       # Base speed (0 to 1)
KP = 0.15              # Proportional constant for line following
NODE_DETECT_DELAY = 500  # ms to wait at node
BOX_PICKUP_TIME = 2000   # Time to hold electromagnet on (ms)

# Obstacle detection constants
OBSTACLE_DETECTION_DISTANCE = 150  # mm
BOX_DETECTION_DISTANCE = 100       # mm (closer than obstacle distance)
AVOIDANCE_MOVE_DISTANCE = 300      # mm to move during avoidance
AVOIDANCE_DELAY = 500              # ms between avoidance steps

# PID Controller Constants
Kp = 0.75   # Proportional gain
Ki = 0.001  # Integral gain
Kd = 0.08   # Derivative gain
integral = 0
prev_err = 0

# Movement Parameters
TURN_90 = 44         # Step count for 90° turn
TURN_180 = 88        # Step count for 180° turn
TURN_INTERSECTION = 10  # Steps to exit intersection
MAX_SPEED = 6.28     # Max wheel velocity (radians/sec)
TURN_SPEED = 0.25 * MAX_SPEED  # Turning speed
LINE_FOLLOW_SPEED = 0.3 * MAX_SPEED  # Line following speed

# ==================== MISSION PARAMETERS ====================
START_NODE = "7A"  # Starting position on map
TARGET_NODES = ["3B", "4C", "2F", "1D"]  # Box pickup locations
DROPOFF_NODES = ["6A", "6D", "6F", "7D"]  # Box delivery locations

# ==================== NODE GRAPH DEFINITION ====================
graph = {
    "1A": [("2C", 21, 0)], 
    "1B": [("2D", 21, 0)], 
    "1C": [("2E", 21, 0)], 
    "1D": [("2F", 21, 0)],
    "2A": [("2B", 49, 1), ("3A", 19, 0)],
    "2B": [("2A", 49, 3), ("3B", 19, 0), ("2C", 50, 1)],
    "2C": [("1A", 21, 2), ("2D", 12, 1), ("2B", 50, 3)],
    "2D": [("2C", 12, 3), ("1B", 21, 2), ("2E", 12, 1)],
    "2E": [("2D", 12, 3), ("1C", 21, 2), ("2F", 12, 1)],
    "2F": [("2E", 12, 3), ("1D", 21, 2), ("4C", 34, 1)],
    "3A": [("2A", 19, 2), ("4A", 13, 0), ("3B", 72, 1)],
    "3B": [("3A", 72, 3), ("2B", 19, 2), ("4B", 13, 0)],
    "4A": [("3A", 13, 2), ("4B", 49, 1), ("6A", 34, 0)],
    "4B": [("3B", 13, 2), ("4A", 49, 3), ("5A", 12, 0), ("4C", 49, 1)],
    "4C": [("2F", 34, 2), ("4B", 49, 3), ("5B", 13, 0)],
    "5A": [("4B", 12, 2), ("5B", 49, 1), ("6E", 13, 0)],
    "5B": [("5A", 49, 3), ("4C", 8, 2), ("6F", 19, 0)],
    "6A": [("4A", 34, 2), ("6B", 12, 1), ("7A", 12, 0)],
    "6B": [("6C", 12, 1), ("7B", 12, 0), ("6A", 12, 3)],
    "6C": [("6D", 12, 1), ("7C", 21, 0), ("6B", 12, 3)],
    "6D": [("6E", 12, 1), ("7D", 12, 0), ("6C", 12, 3)],
    "6E": [("5A", 19, 2), ("6F", 49, 1), ("6D", 13, 3)],
    "6F": [("5B", 19, 2), ("6E", 49, 3)],
    "7A": [("6A", 12, 2)],
    "7B": [("6B", 12, 2)],
    "7C": [("6C", 12, 2)],
    "7D": [("6D", 12, 2)]
}

# Special turn adjustments for specific intersections
ADJUSTED_TURNS = {
    ("4B", "5A"): 39,
    ("2B", "3B"): 49,
    ("5A", "5B"): 50,
    ("6A", "6B"): 49,
    ("6D", "7D"): 49,
    ("6E", "5A"): 44,
    ("4A", "4B"): 44,
    ("4B", "3B"): 54
}

# ==================== STATE MACHINE DEFINITIONS ====================
class State:
    FOLLOW_LINE = 0     # Following line normally
    TURN_PREP = 1       # Preparing for a turn
    IN_TURN = 2         # Executing a turn
    SEARCH = 3          # Searching for lost line
    INTERSECTION = 4    # Handling intersection
    APPROACH_BOX = 5    # Approaching box at target node
    COLLECT_BOX = 6     # Collecting box with electromagnet
    ROTATE_TO_NEXT_FACE = 7  # Rotating to next face for collection
    ROTATE_TO_NEXT_LOADED_FACE = 8  # Rotating to next loaded face for delivery
    DROP_BOX = 9        # Dropping box at delivery node
    COMPLETED = 10      # Mission complete

class AvoidanceState:
    NONE = 0
    BACKUP = 1
    TURN = 2
    FORWARD = 3
    REPLAN = 4

# ==================== GLOBAL VARIABLES ====================
state = State.FOLLOW_LINE
avoidance_state = AvoidanceState.NONE

full_path = []
current_path_index = 0
processing_targets = True

# Box handling variables
current_face = 0  # Current active face (0-3)
boxes_collected = 0
boxes_delivered = 0
boxes_loaded = [False, False, False, False]
action_start_time = 0
avoidance_start_time = 0

# Navigation variables
next_node = ""
original_target_node = ""
last_obstacle_node = ""
turn_counter = 0
turn_direction = 0
turn_steps_target = 0
turn_lost_line = False
intersection_cooldown = 0
intersection_cooldown_steps = 32
past_intersection = False

# ==================== TRACKNODE CLASS DEFINITION ====================
class TrackNode:
    def __init__(self, graph, start_node, start_direction):
        self.graph = graph
        self.current_node = start_node
        self.direction = start_direction
    
    def get_turn_to(self, next_node):
        for neighbour in self.graph[self.current_node]:
            if neighbour[0] == next_node:
                relative_direction = (neighbour[2] - self.direction) % 4
                return (relative_direction, neighbour[2])
        print("No path found from {} to {}".format(self.current_node, next_node))
        return (0, 0)
    
    def advance(self, next_node):
        turn_info = self.get_turn_to(next_node)
        relative_direction = turn_info[0]
        absolute_direction = turn_info[1]
        self.current_node = next_node
        self.direction = absolute_direction
        return relative_direction

# Initialize node tracking
node_tracking = TrackNode(graph, START_NODE, 2)  # Start facing South

# ==================== TOF SENSOR CLASS ====================
class VL6180X:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
    
    def init(self):
        # Initialize the sensor
        pass
    
    def configureDefault(self):
        # Configure default settings
        pass
    
    def setTimeout(self, timeout):
        # Set timeout
        pass
    
    def readRangeSingleMillimeters(self):
        # Read distance in mm
        # This is a placeholder - implement actual TOF sensor reading
        return -1

# Initialize I2C and TOF sensors
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
tof1 = VL6180X(i2c, TOF1_ADDRESS)
tof2 = VL6180X(i2c, TOF2_ADDRESS)
tof3 = VL6180X(i2c, TOF3_ADDRESS)
tof4 = VL6180X(i2c, TOF4_ADDRESS)

# ==================== MAIN FUNCTIONS ====================
def setup():
    global node_tracking
    
    # Initialize motor PWMs
    LEFT_MOTOR_PWM.freq(1000)
    RIGHT_MOTOR_PWM.freq(1000)
    
    # Initialize electromagnets (active HIGH)
    EMAG1.off()
    EMAG2.off()
    EMAG3.off()
    EMAG4.off()
    
    # Initialize mechano wheels
    MECHANO_ENGAGE.on()  # Engage mechano wheels
    
    # Configure TOF sensors
    tof1.init()
    tof2.init()
    tof3.init()
    tof4.init()
    
    # Initialize mission tracking
    planMission()
    
    print("\n=== Robot Initialization Complete ===")
    printMissionStatus()

def loop():
    global state, avoidance_state, current_path_index, next_node, processing_targets
    global boxes_collected, boxes_delivered, current_face, action_start_time
    global avoidance_start_time, turn_counter, turn_direction, turn_steps_target
    global turn_lost_line, intersection_cooldown, past_intersection
    global integral, prev_err
    
    # ==================== SENSOR READING ====================
    s1 = not S1.value()
    s2 = not S2.value()
    s3 = not S3.value()
    s4 = not S4.value()
    s5 = not S5.value()
    
    line_left = s1
    line_center = s3
    line_right = s5
    intersection_now = (s1 + s3 + s5) > 1
    
    # Update intersection marker
    if not intersection_now:
        past_intersection = True
    
    # ==================== OBSTACLE HANDLING ====================
    handleObstacleAvoidance()
    
    # Only do normal processing if not in avoidance
    if avoidance_state == AvoidanceState.NONE:
        # ==================== STATE MACHINE ====================
        if state == State.FOLLOW_LINE:
            followLine(line_left, line_center, line_right)
            
            if intersection_cooldown == 0 and intersection_now and past_intersection:
                if current_path_index + 1 < len(full_path):
                    next_node = full_path[current_path_index + 1]
                    turn_info = node_tracking.get_turn_to(next_node)
                    turn_direction = turn_info[0]
                    past_intersection = False
                    
                    # Check if we're arriving at a target or dropoff node
                    if processing_targets and next_node == TARGET_NODES[boxes_collected]:
                        state = State.APPROACH_BOX
                        action_start_time = time.ticks_ms()
                        print("Approaching target box...")
                    elif not processing_targets and next_node == DROPOFF_NODES[boxes_delivered]:
                        state = State.DROP_BOX
                        action_start_time = time.ticks_ms()
                        print("Arrived at dropoff location...")
                    else:
                        state = State.TURN_PREP if turn_direction != 0 else State.INTERSECTION
                    turn_counter = 0
                    turn_lost_line = False
        
        elif state == State.APPROACH_BOX:
            tof_distance = getCurrentFaceTOFDistance()
            
            # If we lose the box, go back to following line
            if tof_distance <= 0 or tof_distance > BOX_DETECTION_DISTANCE * 2:
                print("Lost target box, resuming navigation")
                state = State.FOLLOW_LINE
            
            # Slowly approach the box
            elif tof_distance > 50:  # 50mm = 5cm
                setMotorSpeeds(0.15 * MAX_SPEED, 0.15 * MAX_SPEED)
            else:
                setMotorSpeeds(0, 0)
                state = State.COLLECT_BOX
                action_start_time = time.ticks_ms()
        
        elif state == State.COLLECT_BOX:
            # Activate current face's electromagnet
            activateElectromagnet(True)
            
            if time.ticks_diff(time.ticks_ms(), action_start_time) > BOX_PICKUP_TIME:
                boxes_loaded[current_face] = True
                boxes_collected += 1
                activateElectromagnet(False)
                printMissionStatus()
                
                if boxes_collected < 4:
                    # Rotate to next face for next collection
                    state = State.ROTATE_TO_NEXT_FACE
                    action_start_time = time.ticks_ms()
                    print("Rotating to next face...")
                else:
                    # All boxes collected, proceed to dropoffs
                    processing_targets = False
                    current_path_index += 1
                    planMission()
                    state = State.INTERSECTION
                    print("All boxes collected! Moving to dropoff locations...")
        
        elif state == State.DROP_BOX:
            # Deactivate electromagnet to release box
            activateElectromagnet(False)
            
            if time.ticks_diff(time.ticks_ms(), action_start_time) > 500:  # Brief pause for drop
                boxes_loaded[current_face] = False
                boxes_delivered += 1
                printMissionStatus()
                
                if boxes_delivered < 4:
                    # Rotate to next loaded face
                    state = State.ROTATE_TO_NEXT_LOADED_FACE
                    action_start_time = time.ticks_ms()
                    print("Rotating to next loaded face...")
                else:
                    # All boxes delivered
                    state = State.COMPLETED
                    print("All boxes delivered!")
        
        elif state == State.ROTATE_TO_NEXT_FACE:
            # Rotate 90 degrees to next face (for collection)
            current_face = (current_face + 1) % 4
            setMotorSpeeds(TURN_SPEED, -TURN_SPEED)
            
            if time.ticks_diff(time.ticks_ms(), action_start_time) > TURN_90:
                setMotorSpeeds(0, 0)
                state = State.FOLLOW_LINE
                print("Now using face {} for next operation".format(current_face + 1))
        
        elif state == State.ROTATE_TO_NEXT_LOADED_FACE:
            # Find next face that has a box
            for i in range(1, 5):
                next_face = (current_face + i) % 4
                if boxes_loaded[next_face]:
                    current_face = next_face
                    break
            setMotorSpeeds(TURN_SPEED, -TURN_SPEED)
            
            if time.ticks_diff(time.ticks_ms(), action_start_time) > TURN_90:
                setMotorSpeeds(0, 0)
                state = State.FOLLOW_LINE
                print("Now using face {} for delivery".format(current_face + 1))
        
        elif state == State.TURN_PREP:
            setMotorSpeeds(LINE_FOLLOW_SPEED, LINE_FOLLOW_SPEED)
            turn_counter += 1
            if turn_counter >= 4:
                state = State.IN_TURN
                turn_counter = 0
                # Use adjusted turn if specified, otherwise default
                key = (node_tracking.current_node, next_node)
                if key in ADJUSTED_TURNS:
                    turn_steps_target = ADJUSTED_TURNS[key]
                else:
                    turn_steps_target = TURN_90 if turn_direction in (1, 3) else TURN_180
        
        elif state == State.IN_TURN:
            if turn_direction == 1:  # Right turn
                setMotorSpeeds(TURN_SPEED, -TURN_SPEED)
            elif turn_direction == 3:  # Left turn
                setMotorSpeeds(-TURN_SPEED, TURN_SPEED)
            elif turn_direction == 2:  # About face
                setMotorSpeeds(TURN_SPEED, -TURN_SPEED)
            
            turn_counter += 1
            if not (line_left or line_center or line_right):
                turn_lost_line = True
            if (turn_lost_line and line_center) or (turn_counter >= turn_steps_target):
                state = State.SEARCH
                turn_counter = 0
                turn_lost_line = False
        
        elif state == State.SEARCH:
            setMotorSpeeds(0.5 * MAX_SPEED, 0.5 * MAX_SPEED)
            if line_center:
                state = State.INTERSECTION
                turn_counter = 0
        
        elif state == State.INTERSECTION:
            setMotorSpeeds(LINE_FOLLOW_SPEED, LINE_FOLLOW_SPEED)
            turn_counter += 1
            if turn_counter >= TURN_INTERSECTION and not intersection_now:
                node_tracking.advance(next_node)
                current_path_index += 1
                intersection_cooldown = intersection_cooldown_steps
                past_intersection = False
                state = State.FOLLOW_LINE
                turn_counter = 0
        
        elif state == State.COMPLETED:
            setMotorSpeeds(0, 0)
            MECHANO_ENGAGE.off()  # Disengage mechano wheels
            while True:
                print("=== MISSION COMPLETE ===")
                print("All boxes delivered successfully!")
                time.sleep(1)
        
        # Small obstacle avoidance (quick adjustment)
        distance = getCurrentFaceTOFDistance()
        if (distance > 0 and distance < 150 and 
            not isApproachingTargetBox() and
            state != State.APPROACH_BOX and 
            state != State.COLLECT_BOX and 
            state != State.DROP_BOX):
            avoidObstacle()
    
    time.sleep_ms(10)  # Small delay for stability

# ==================== CORE FUNCTION IMPLEMENTATIONS ====================
def handleObstacleAvoidance():
    global state, avoidance_state, next_node, original_target_node
    global last_obstacle_node, action_start_time, avoidance_start_time
    
    tof_distance = getCurrentFaceTOFDistance()
    
    # Don't avoid during box operations or when not moving forward
    if state in (State.APPROACH_BOX, State.COLLECT_BOX, State.DROP_BOX) or state != State.FOLLOW_LINE:
        return
    
    # Special case: if we're heading to a target node and see something close
    if (processing_targets and 
        next_node == TARGET_NODES[boxes_collected] and 
        tof_distance < BOX_DETECTION_DISTANCE):
        # This is likely the target box - don't avoid it
        print("Target box detected - preparing for pickup")
        state = State.APPROACH_BOX
        avoidance_state = AvoidanceState.NONE
        action_start_time = time.ticks_ms()
        return
    
    # Regular obstacle detection
    if tof_distance > 0 and tof_distance < OBSTACLE_DETECTION_DISTANCE:
        if avoidance_state == AvoidanceState.NONE:
            # Only avoid if we're not approaching our target box
            if not (processing_targets and next_node == TARGET_NODES[boxes_collected]):
                print("Obstacle detected! Starting avoidance maneuver...")
                avoidance_state = AvoidanceState.BACKUP
                avoidance_start_time = time.ticks_ms()
                original_target_node = next_node
    
    # Execute avoidance maneuver
    if avoidance_state == AvoidanceState.BACKUP:
        setMotorSpeeds(-0.3 * MAX_SPEED, -0.3 * MAX_SPEED)
        if time.ticks_diff(time.ticks_ms(), avoidance_start_time) > AVOIDANCE_DELAY:
            avoidance_state = AvoidanceState.TURN
            avoidance_start_time = time.ticks_ms()
    
    elif avoidance_state == AvoidanceState.TURN:
        setMotorSpeeds(0.4 * MAX_SPEED, -0.4 * MAX_SPEED)
        if time.ticks_diff(time.ticks_ms(), avoidance_start_time) > AVOIDANCE_DELAY:
            avoidance_state = AvoidanceState.FORWARD
            avoidance_start_time = time.ticks_ms()
    
    elif avoidance_state == AvoidanceState.FORWARD:
        setMotorSpeeds(0.4 * MAX_SPEED, 0.4 * MAX_SPEED)
        if time.ticks_diff(time.ticks_ms(), avoidance_start_time) > AVOIDANCE_DELAY:
            avoidance_state = AvoidanceState.REPLAN
    
    elif avoidance_state == AvoidanceState.REPLAN:
        setMotorSpeeds(0, 0)
        print("Attempting to replan path...")
        
        # Mark this node as having an obstacle
        last_obstacle_node = node_tracking.current_node
        
        # Replan path to original target
        planMission()
        
        # Reset state
        avoidance_state = AvoidanceState.NONE
        state = State.FOLLOW_LINE

def planMission():
    global full_path, current_path_index, processing_targets
    
    mission_path = []
    current = node_tracking.current_node
    target = TARGET_NODES[boxes_collected] if processing_targets else DROPOFF_NODES[boxes_delivered]
    
    # Try to find path avoiding last known obstacle
    path_segment = dijkstra(current, target)
    
    # If no path found or path goes through obstacle node, try alternative
    if not path_segment or (last_obstacle_node and last_obstacle_node in path_segment):
        print("Standard path blocked, trying alternatives...")
        
        # Try to find path to any adjacent node first
        for neighbor in graph[current]:
            intermediate = neighbor[0]
            if intermediate != last_obstacle_node:
                path_to_neighbor = dijkstra(current, intermediate)
                path_from_neighbor = dijkstra(intermediate, target)
                
                if path_to_neighbor and path_from_neighbor:
                    path_segment = path_to_neighbor + path_from_neighbor[1:]
                    break
    
    if not path_segment:
        print("No valid path found! Waiting...")
        setMotorSpeeds(0, 0)
        time.sleep(5)  # Wait 5 seconds and try again
        planMission()
        return
    
    full_path = path_segment
    current_path_index = 0
    
    print("New path: " + " ".join(full_path))

def dijkstra(start, goal):
    # Priority queue: (total_distance, current_node)
    pq = []
    heapq.heappush(pq, (0, start))
    visited = set()
    previous = {}
    distances = defaultdict(lambda: float('inf'))
    distances[start] = 0
    
    while pq:
        current_dist, current = heapq.heappop(pq)
        
        if current == goal:
            break
        if current in visited:
            continue
        
        visited.add(current)
        
        for neighbor in graph[current]:
            next_node = neighbor[0]
            weight = neighbor[1]
            
            if distances[next_node] > distances[current] + weight:
                distances[next_node] = distances[current] + weight
                previous[next_node] = current
                heapq.heappush(pq, (distances[next_node], next_node))
    
    # Reconstruct path
    path = []
    current = goal
    while current != start:
        path.insert(0, current)
        current = previous.get(current, start)
    path.insert(0, start)
    
    return path

def isApproachingTargetBox():
    return (processing_targets and 
            full_path and 
            current_path_index + 1 < len(full_path) and
            full_path[current_path_index + 1] == TARGET_NODES[boxes_collected])

def getCurrentFaceTOFDistance():
    if current_face == 0:
        return tof1.readRangeSingleMillimeters()
    elif current_face == 1:
        return tof2.readRangeSingleMillimeters()
    elif current_face == 2:
        return tof3.readRangeSingleMillimeters()
    elif current_face == 3:
        return tof4.readRangeSingleMillimeters()
    return -1

def activateElectromagnet(on):
    if current_face == 0:
        EMAG1.value(on)
    elif current_face == 1:
        EMAG2.value(on)
    elif current_face == 2:
        EMAG3.value(on)
    elif current_face == 3:
        EMAG4.value(on)

def followLine(line_left, line_center, line_right):
    global integral, prev_err
    
    error = 0
    
    if line_left and not line_center and not line_right:
        error = -1
    elif line_right and not line_center and not line_left:
        error = 1
    elif line_center and not line_left and not line_right:
        error = 0
    elif line_left and line_center and not line_right:
        error = -0.5
    elif line_right and line_center and not line_left:
        error = 0.5
    else:
        error = 0
    
    correction = pid_calc(error)
    setMotorSpeeds(LINE_FOLLOW_SPEED - correction, LINE_FOLLOW_SPEED + correction)

def pid_calc(error):
    global integral, prev_err
    
    integral += error
    derivative = error - prev_err
    prev_err = error
    return Kp * error + Ki * integral + Kd * derivative

def avoidObstacle():
    print("Small obstacle detected, quick avoidance...")
    
    # Back up slightly
    setMotorSpeeds(-0.3 * MAX_SPEED, -0.3 * MAX_SPEED)
    time.sleep_ms(300)
    
    # Turn slightly
    setMotorSpeeds(0.3 * MAX_SPEED, -0.3 * MAX_SPEED)
    time.sleep_ms(200)
    
    # Continue forward
    setMotorSpeeds(0.5 * MAX_SPEED, 0.5 * MAX_SPEED)
    time.sleep_ms(300)

def setMotorSpeeds(left, right):
    # Constrain speeds to valid range
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    
    # Set left motor direction and speed
    LEFT_MOTOR_DIR.value(1 if left > 0 else 0)
    LEFT_MOTOR_PWM.duty_u16(int(abs(left) * 65535 / MAX_SPEED))
    
    # Set right motor direction and speed
    RIGHT_MOTOR_DIR.value(1 if right > 0 else 0)
    RIGHT_MOTOR_PWM.duty_u16(int(abs(right) * 65535 / MAX_SPEED))

def printMissionStatus():
    print("\n=== MISSION STATUS ===")
    if processing_targets:
        print("Phase: COLLECTING | Boxes: {}/4".format(boxes_collected))
    else:
        print("Phase: DELIVERING | Boxes: {}/4".format(boxes_delivered))
    
    print("Current face: {} | Loaded faces: {}".format(
        current_face + 1, " ".join("1" if loaded else "0" for loaded in boxes_loaded)))
    print("Current node: {}".format(node_tracking.current_node))
    print("=====================")

# ==================== MAIN PROGRAM ====================
if __name__ == "__main__":
    setup()
    while True:
        loop()
