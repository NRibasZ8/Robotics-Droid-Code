/* TO DO:
** Add single sided joints (apart from full intersections)
** Make "lef" and "right" of line sensors count on two sensors instead of just one (left = s1 + s2, not just s1)
** Use Gadiel's code for the mechano wheels (?)
**Test lol
*/

#include <Wire.h>
#include <VL6180X.h>
#include <Arduino.h>
#include <vector>
#include <queue>
#include <map>

// ==================== HARDWARE CONFIGURATION ====================
// Line Sensor Pins
const int S1 = 13;  // Far left sensor
const int S2 = 12;  // Middle left sensor
const int S3 = 14;  // Center sensor
const int S4 = 27;  // Middle right sensor
const int S5 = 26;  // Far right sensor

// Limit Switch and Proximity Sensor
const int CLP = 25;    // Limit switch (with pullup)
const int NEAR = 33;   // Proximity sensor

// Motor Control Pins
const int LEFT_MOTOR_PWM = 32;
const int LEFT_MOTOR_DIR = 23;
const int RIGHT_MOTOR_PWM = 18;
const int RIGHT_MOTOR_DIR = 19;

// Electromagnet Control Pins (one per face)
const int EMAG1 = 15;  // Face 1 electromagnet
const int EMAG2 = 2;   // Face 2 electromagnet
const int EMAG3 = 4;   // Face 3 electromagnet
const int EMAG4 = 5;   // Face 4 electromagnet

// Mechano Wheel Control
const int MECHANO_ENGAGE = 17;  // HIGH = engaged, LOW = disengaged

// TOF Sensor I2C Addresses
const uint8_t TOF1_ADDRESS = 0x30;  // Face 1 TOF
const uint8_t TOF2_ADDRESS = 0x31;  // Face 2 TOF
const uint8_t TOF3_ADDRESS = 0x32;  // Face 3 TOF
const uint8_t TOF4_ADDRESS = 0x33;  // Face 4 TOF

// TOF Sensor Objects
VL6180X tof1;
VL6180X tof2;
VL6180X tof3;
VL6180X tof4;

// ==================== NAVIGATION CONSTANTS ====================
const float BASE_SPEED = 0.6;       // Base speed (0 to 1)
const float KP = 0.15;              // Proportional constant for line following
const unsigned long NODE_DETECT_DELAY = 500;  // ms to wait at node
const unsigned long BOX_PICKUP_TIME = 2000;   // Time to hold electromagnet on (ms)

// Obstacle detection constants
const int OBSTACLE_DETECTION_DISTANCE = 150; // mm
const int BOX_DETECTION_DISTANCE = 100;      // mm (closer than obstacle distance)
const int AVOIDANCE_MOVE_DISTANCE = 300;     // mm to move during avoidance
const unsigned long AVOIDANCE_DELAY = 500;   // ms between avoidance steps

// PID Controller Constants
const float Kp = 0.75;  // Proportional gain
const float Ki = 0.001; // Integral gain
const float Kd = 0.08;  // Derivative gain
float integral = 0;
float prev_err = 0;

// Movement Parameters
const int TURN_90 = 44;        // Step count for 90° turn
const int TURN_180 = 88;       // Step count for 180° turn
const int TURN_INTERSECTION = 10; // Steps to exit intersection
const float MAX_SPEED = 6.28;  // Max wheel velocity (radians/sec)
const float TURN_SPEED = 0.25 * MAX_SPEED;  // Turning speed
const float LINE_FOLLOW_SPEED = 0.3 * MAX_SPEED; // Line following speed

// ==================== MISSION PARAMETERS ====================
const String START_NODE = "7A";  // Starting position on map
const std::vector<String> TARGET_NODES = {"3B", "4C", "2F", "1D"};  // Box pickup locations
const std::vector<String> DROPOFF_NODES = {"6A", "6D", "6F", "7D"}; // Box delivery locations

// ==================== NODE GRAPH DEFINITION ====================
std::map<String, std::vector<std::tuple<String, int, int>>> graph = {
    {"1A", {{"2C", 21, 0}}}, 
    {"1B", {{"2D", 21, 0}}}, 
    {"1C", {{"2E", 21, 0}}}, 
    {"1D", {{"2F", 21, 0}}},
    {"2A", {{"2B", 49, 1}, {"3A", 19, 0}}},
    {"2B", {{"2A", 49, 3}, {"3B", 19, 0}, {"2C", 50, 1}}},
    {"2C", {{"1A", 21, 2}, {"2D", 12, 1}, {"2B", 50, 3}}},
    {"2D", {{"2C", 12, 3}, {"1B", 21, 2}, {"2E", 12, 1}}},
    {"2E", {{"2D", 12, 3}, {"1C", 21, 2}, {"2F", 12, 1}}},
    {"2F", {{"2E", 12, 3}, {"1D", 21, 2}, {"4C", 34, 1}}},
    {"3A", {{"2A", 19, 2}, {"4A", 13, 0}, {"3B", 72, 1}}},
    {"3B", {{"3A", 72, 3}, {"2B", 19, 2}, {"4B", 13, 0}}},
    {"4A", {{"3A", 13, 2}, {"4B", 49, 1}, {"6A", 34, 0}}},
    {"4B", {{"3B", 13, 2}, {"4A", 49, 3}, {"5A", 12, 0}, {"4C", 49, 1}}},
    {"4C", {{"2F", 34, 2}, {"4B", 49, 3}, {"5B", 13, 0}}},
    {"5A", {{"4B", 12, 2}, {"5B", 49, 1}, {"6E", 13, 0}}},
    {"5B", {{"5A", 49, 3}, {"4C", 8, 2}, {"6F", 19, 0}}},
    {"6A", {{"4A", 34, 2}, {"6B", 12, 1}, {"7A", 12, 0}}},
    {"6B", {{"6C", 12, 1}, {"7B", 12, 0}, {"6A", 12, 3}}},
    {"6C", {{"6D", 12, 1}, {"7C", 21, 0}, {"6B", 12, 3}}},
    {"6D", {{"6E", 12, 1}, {"7D", 12, 0}, {"6C", 12, 3}}},
    {"6E", {{"5A", 19, 2}, {"6F", 49, 1}, {"6D", 13, 3}}},
    {"6F", {{"5B", 19, 2}, {"6E", 49, 3}}},
    {"7A", {{"6A", 12, 2}}},
    {"7B", {{"6B", 12, 2}}},
    {"7C", {{"6C", 12, 2}}},
    {"7D", {{"6D", 12, 2}}}
};

// Special turn adjustments for specific intersections
std::map<std::pair<String, String>, int> ADJUSTED_TURNS = {
    {{"4B", "5A"}, 39},
    {{"2B", "3B"}, 49},
    {{"5A", "5B"}, 50},
    {{"6A", "6B"}, 49},
    {{"6D", "7D"}, 49},
    {{"6E", "5A"}, 44},
    {{"4A", "4B"}, 44},
    {{"4B", "3B"}, 54}
};

// ==================== STATE MACHINE DEFINITIONS ====================
enum State { 
    FOLLOW_LINE,    // Following line normally
    TURN_PREP,      // Preparing for a turn
    IN_TURN,        // Executing a turn
    SEARCH,         // Searching for lost line
    INTERSECTION,   // Handling intersection
    APPROACH_BOX,   // Approaching box at target node
    COLLECT_BOX,    // Collecting box with electromagnet
    ROTATE_TO_NEXT_FACE, // Rotating to next face for collection
    ROTATE_TO_NEXT_LOADED_FACE, // Rotating to next loaded face for delivery
    DROP_BOX,       // Dropping box at delivery node
    COMPLETED       // Mission complete
};

enum AvoidanceState {
    AVOIDANCE_NONE,
    AVOIDANCE_BACKUP,
    AVOIDANCE_TURN,
    AVOIDANCE_FORWARD,
    AVOIDANCE_REPLAN
};

// ==================== GLOBAL VARIABLES ====================
State state = FOLLOW_LINE;
AvoidanceState avoidance_state = AVOIDANCE_NONE;

std::vector<String> full_path;
size_t current_path_index = 0;
bool processing_targets = true;

// Box handling variables
int current_face = 0;  // Current active face (0-3)
int boxes_collected = 0;
int boxes_delivered = 0;
bool boxes_loaded[4] = {false, false, false, false};
unsigned long action_start_time = 0;
unsigned long avoidance_start_time = 0;

// Navigation variables
String next_node = "";
String original_target_node = "";
String last_obstacle_node = "";
int turn_counter = 0;
int turn_direction = 0;
int turn_steps_target = 0;
bool turn_lost_line = false;
int intersection_cooldown = 0;
const int intersection_cooldown_steps = 32;
bool past_intersection = false;

// ==================== FUNCTION PROTOTYPES ====================
// Navigation functions
std::vector<String> dijkstra(String start, String goal);
void followLine(bool line_left, bool line_center, bool line_right);
float pid_calc(float error);
void avoidObstacle();
void setMotorSpeeds(float left, float right);
void handleObstacleAvoidance();

// Mission control functions
void planMission();
void printMissionStatus();
bool isApproachingTargetBox();

// Hardware control functions
void configureTOF(VL6180X& sensor, uint8_t address);
int getCurrentFaceTOFDistance();
void activateElectromagnet(bool on);

// ==================== TRACKNODE CLASS DEFINITION ====================
class TrackNode {
public:
    String current_node;
    int direction;
    
    TrackNode(std::map<String, std::vector<std::tuple<String, int, int>>> graph, 
              String start_node, int start_direction) {
        this->graph = graph;
        this->current_node = start_node;
        this->direction = start_direction;
    }
    
    std::pair<int, int> get_turn_to(String next_node) {
        for (auto& neighbour : graph[current_node]) {
            if (std::get<0>(neighbour) == next_node) {
                int relative_direction = (std::get<2>(neighbour) - direction) % 4;
                return {relative_direction, std::get<2>(neighbour)};
            }
        }
        Serial.printf("No path found from %s to %s\n", current_node.c_str(), next_node.c_str());
        return {0, 0};
    }
    
    int advance(String next_node) {
        auto turn_info = get_turn_to(next_node);
        int relative_direction = turn_info.first;
        int absolute_direction = turn_info.second;
        current_node = next_node;
        direction = absolute_direction;
        return relative_direction;
    }

private:
    std::map<String, std::vector<std::tuple<String, int, int>>> graph;
};

//Declare the node_tracking pointer after the class definition
TrackNode* node_tracking;

// ==================== MAIN ARDUINO FUNCTIONS ====================
void setup() {
    Serial.begin(115200);
    
    // Initialize sensor pins
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);
    pinMode(CLP, INPUT_PULLUP);
    pinMode(NEAR, INPUT);
    
    // Initialize motor control pins
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
    
    // Initialize electromagnets (active HIGH)
    pinMode(EMAG1, OUTPUT);
    pinMode(EMAG2, OUTPUT);
    pinMode(EMAG3, OUTPUT);
    pinMode(EMAG4, OUTPUT);
    digitalWrite(EMAG1, LOW);
    digitalWrite(EMAG2, LOW);
    digitalWrite(EMAG3, LOW);
    digitalWrite(EMAG4, LOW);
    
    // Initialize mechano wheels
    pinMode(MECHANO_ENGAGE, OUTPUT);
    digitalWrite(MECHANO_ENGAGE, HIGH);  // Engage mechano wheels
    
    // Initialize I2C for TOF sensors
    Wire.begin(21, 22);
    delay(100);
    
    // Configure TOF sensors with unique addresses
    configureTOF(tof1, TOF1_ADDRESS);
    configureTOF(tof2, TOF2_ADDRESS);
    configureTOF(tof3, TOF3_ADDRESS);
    configureTOF(tof4, TOF4_ADDRESS);
    
    // Initialize mission tracking
    node_tracking = new TrackNode(graph, START_NODE, 2);  // Start facing South
    planMission();
    
    Serial.println("\n=== Robot Initialization Complete ===");
    printMissionStatus();
}

void loop() {
    // ==================== SENSOR READING ====================
    bool s1 = !digitalRead(S1);
    bool s2 = !digitalRead(S2);
    bool s3 = !digitalRead(S3);
    bool s4 = !digitalRead(S4);
    bool s5 = !digitalRead(S5);
    
    bool line_left = s1;
    bool line_center = s3;
    bool line_right = s5;
    bool intersection_now = (s1 + s3 + s5) > 1;
    
    // Update intersection marker
    if (!intersection_now) {
        past_intersection = true;
    }
    
    // ==================== OBSTACLE HANDLING ====================
    handleObstacleAvoidance();
    
    // Only do normal processing if not in avoidance
    if (avoidance_state == AVOIDANCE_NONE) {
        // ==================== STATE MACHINE ====================
        switch(state) {
            // ---------- Line Following State ----------
            case FOLLOW_LINE:
                followLine(line_left, line_center, line_right);
                
                if (intersection_cooldown == 0 && intersection_now && past_intersection) {
                    if (current_path_index + 1 < full_path.size()) {
                        next_node = full_path[current_path_index + 1];
                        auto turn_info = node_tracking->get_turn_to(next_node);
                        turn_direction = turn_info.first;
                        past_intersection = false;
                        
                        // Check if we're arriving at a target or dropoff node
                        if (processing_targets && next_node == TARGET_NODES[boxes_collected]) {
                            state = APPROACH_BOX;
                            action_start_time = millis();
                            Serial.println("Approaching target box...");
                        } 
                        else if (!processing_targets && next_node == DROPOFF_NODES[boxes_delivered]) {
                            state = DROP_BOX;
                            action_start_time = millis();
                            Serial.println("Arrived at dropoff location...");
                        } 
                        else {
                            state = (turn_direction != 0) ? TURN_PREP : INTERSECTION;
                        }
                        turn_counter = 0;
                        turn_lost_line = false;
                    }
                }
                break;
                
            // ---------- Box Approach State ----------
            case APPROACH_BOX: {
                int tof_distance = getCurrentFaceTOFDistance();
                
                // If we lose the box, go back to following line
                if (tof_distance <= 0 || tof_distance > BOX_DETECTION_DISTANCE * 2) {
                    Serial.println("Lost target box, resuming navigation");
                    state = FOLLOW_LINE;
                    break;
                }
                
                // Slowly approach the box
                if (tof_distance > 50) {  // 50mm = 5cm
                    setMotorSpeeds(0.15 * MAX_SPEED, 0.15 * MAX_SPEED);
                } else {
                    setMotorSpeeds(0, 0);
                    state = COLLECT_BOX;
                    action_start_time = millis();
                }
                break;
            }
                
            // ---------- Box Collection State ----------
            case COLLECT_BOX:
                // Activate current face's electromagnet
                activateElectromagnet(true);
                
                if (millis() - action_start_time > BOX_PICKUP_TIME) {
                    boxes_loaded[current_face] = true;
                    boxes_collected++;
                    activateElectromagnet(false);
                    printMissionStatus();
                    
                    if (boxes_collected < 4) {
                        // Rotate to next face for next collection
                        state = ROTATE_TO_NEXT_FACE;
                        action_start_time = millis();
                        Serial.println("Rotating to next face...");
                    } 
                    else {
                        // All boxes collected, proceed to dropoffs
                        processing_targets = false;
                        current_path_index++;
                        planMission();
                        state = INTERSECTION;
                        Serial.println("All boxes collected! Moving to dropoff locations...");
                    }
                }
                break;
                
            // ---------- Box Dropoff State ----------
            case DROP_BOX:
                // Deactivate electromagnet to release box
                activateElectromagnet(false);
                
                if (millis() - action_start_time > 500) {  // Brief pause for drop
                    boxes_loaded[current_face] = false;
                    boxes_delivered++;
                    printMissionStatus();
                    
                    if (boxes_delivered < 4) {
                        // Rotate to next loaded face
                        state = ROTATE_TO_NEXT_LOADED_FACE;
                        action_start_time = millis();
                        Serial.println("Rotating to next loaded face...");
                    } 
                    else {
                        // All boxes delivered
                        state = COMPLETED;
                        Serial.println("All boxes delivered!");
                    }
                }
                break;
                
            // ---------- Face Rotation States ----------
            case ROTATE_TO_NEXT_FACE:
                // Rotate 90 degrees to next face (for collection)
                current_face = (current_face + 1) % 4;
                setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
                
                if (millis() - action_start_time > TURN_90) {
                    setMotorSpeeds(0, 0);
                    state = FOLLOW_LINE;
                    Serial.printf("Now using face %d for next operation\n", current_face + 1);
                }
                break;
                
            case ROTATE_TO_NEXT_LOADED_FACE:
                // Find next face that has a box
                for (int i = 1; i <= 4; i++) {
                    int next_face = (current_face + i) % 4;
                    if (boxes_loaded[next_face]) {
                        current_face = next_face;
                        break;
                    }
                }
                setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
                
                if (millis() - action_start_time > TURN_90) {
                    setMotorSpeeds(0, 0);
                    state = FOLLOW_LINE;
                    Serial.printf("Now using face %d for delivery\n", current_face + 1);
                }
                break;
                
            // ---------- Navigation States ----------
            case TURN_PREP:
                setMotorSpeeds(LINE_FOLLOW_SPEED, LINE_FOLLOW_SPEED);
                if (++turn_counter >= 4) {
                    state = IN_TURN;
                    turn_counter = 0;
                    // Use adjusted turn if specified, otherwise default
                    auto key = std::make_pair(node_tracking->current_node, next_node);
                    if (ADJUSTED_TURNS.find(key) != ADJUSTED_TURNS.end()) {
                        turn_steps_target = ADJUSTED_TURNS[key];
                    } 
                    else {
                        turn_steps_target = (turn_direction == 1 || turn_direction == 3) ? TURN_90 : TURN_180;
                    }
                }
                break;
                
            case IN_TURN:
                if (turn_direction == 1) {  // Right turn
                    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
                } 
                else if (turn_direction == 3) {  // Left turn
                    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
                } 
                else if (turn_direction == 2) {  // About face
                    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
                }
                
                turn_counter++;
                if (!(line_left || line_center || line_right)) {
                    turn_lost_line = true;
                }
                if ((turn_lost_line && line_center) || (turn_counter >= turn_steps_target)) {
                    state = SEARCH;
                    turn_counter = 0;
                    turn_lost_line = false;
                }
                break;
                
            case SEARCH:
                setMotorSpeeds(0.5 * MAX_SPEED, 0.5 * MAX_SPEED);
                if (line_center) {
                    state = INTERSECTION;
                    turn_counter = 0;
                }
                break;
                
            case INTERSECTION:
                setMotorSpeeds(LINE_FOLLOW_SPEED, LINE_FOLLOW_SPEED);
                turn_counter++;
                if (turn_counter >= TURN_INTERSECTION && !intersection_now) {
                    node_tracking->advance(next_node);
                    current_path_index++;
                    intersection_cooldown = intersection_cooldown_steps;
                    past_intersection = false;
                    state = FOLLOW_LINE;
                    turn_counter = 0;
                }
                break;
                
            // ---------- Mission Complete State ----------
            case COMPLETED:
                setMotorSpeeds(0, 0);
                digitalWrite(MECHANO_ENGAGE, LOW);  // Disengage mechano wheels
                while(1) { 
                    Serial.println("=== MISSION COMPLETE ===");
                    Serial.println("All boxes delivered successfully!");
                    delay(1000); 
                }
                break;
        }
        
        // Small obstacle avoidance (quick adjustment)
        float distance = getCurrentFaceTOFDistance();
        if (distance > 0 && distance < 150 && 
            !isApproachingTargetBox() &&
            state != APPROACH_BOX && 
            state != COLLECT_BOX && 
            state != DROP_BOX) {
            avoidObstacle();
        }
    }
    
    delay(10);  // Small delay for stability
}

// ==================== CORE FUNCTION IMPLEMENTATIONS ====================

void handleObstacleAvoidance() {
    int tof_distance = getCurrentFaceTOFDistance();
    
    // Don't avoid during box operations or when not moving forward
    if (state == APPROACH_BOX || state == COLLECT_BOX || 
        state == DROP_BOX || state != FOLLOW_LINE) {
        return;
    }
    
    // Special case: if we're heading to a target node and see something close
    if (processing_targets && 
        next_node == TARGET_NODES[boxes_collected] && 
        tof_distance < BOX_DETECTION_DISTANCE) {
        // This is likely the target box - don't avoid it
        Serial.println("Target box detected - preparing for pickup");
        state = APPROACH_BOX;
        avoidance_state = AVOIDANCE_NONE;
        action_start_time = millis();
        return;
    }
    
    // Regular obstacle detection
    if (tof_distance > 0 && tof_distance < OBSTACLE_DETECTION_DISTANCE) {
        if (avoidance_state == AVOIDANCE_NONE) {
            // Only avoid if we're not approaching our target box
            if (!(processing_targets && next_node == TARGET_NODES[boxes_collected])) {
                Serial.println("Obstacle detected! Starting avoidance maneuver...");
                avoidance_state = AVOIDANCE_BACKUP;
                avoidance_start_time = millis();
                original_target_node = next_node;
            }
        }
    }
    
    // Execute avoidance maneuver
    switch (avoidance_state) {
        case AVOIDANCE_BACKUP:
            setMotorSpeeds(-0.3 * MAX_SPEED, -0.3 * MAX_SPEED);
            if (millis() - avoidance_start_time > AVOIDANCE_DELAY) {
                avoidance_state = AVOIDANCE_TURN;
                avoidance_start_time = millis();
            }
            break;
            
        case AVOIDANCE_TURN:
            setMotorSpeeds(0.4 * MAX_SPEED, -0.4 * MAX_SPEED);
            if (millis() - avoidance_start_time > AVOIDANCE_DELAY) {
                avoidance_state = AVOIDANCE_FORWARD;
                avoidance_start_time = millis();
            }
            break;
            
        case AVOIDANCE_FORWARD:
            setMotorSpeeds(0.4 * MAX_SPEED, 0.4 * MAX_SPEED);
            if (millis() - avoidance_start_time > AVOIDANCE_DELAY) {
                avoidance_state = AVOIDANCE_REPLAN;
            }
            break;
            
        case AVOIDANCE_REPLAN:
            setMotorSpeeds(0, 0);
            Serial.println("Attempting to replan path...");
            
            // Mark this node as having an obstacle
            last_obstacle_node = node_tracking->current_node;
            
            // Replan path to original target
            planMission();
            
            // Reset state
            avoidance_state = AVOIDANCE_NONE;
            state = FOLLOW_LINE;
            break;
            
        case AVOIDANCE_NONE:
        default:
            // No avoidance needed
            break;
    }
}

void planMission() {
    std::vector<String> mission_path;
    String current = node_tracking->current_node;
    String target;
    
    if (processing_targets) {
        target = TARGET_NODES[boxes_collected];
    } else {
        target = DROPOFF_NODES[boxes_delivered];
    }
    
    // Try to find path avoiding last known obstacle
    auto path_segment = dijkstra(current, target);
    
    // If no path found or path goes through obstacle node, try alternative
    if (path_segment.empty() || 
        (last_obstacle_node != "" && 
         std::find(path_segment.begin(), path_segment.end(), last_obstacle_node) != path_segment.end())) {
        
        Serial.println("Standard path blocked, trying alternatives...");
        
        // Try to find path to any adjacent node first
        for (auto& neighbor : graph[current]) {
            String intermediate = std::get<0>(neighbor);
            if (intermediate != last_obstacle_node) {
                auto path_to_neighbor = dijkstra(current, intermediate);
                auto path_from_neighbor = dijkstra(intermediate, target);
                
                if (!path_to_neighbor.empty() && !path_from_neighbor.empty()) {
                    path_segment = path_to_neighbor;
                    path_segment.insert(path_segment.end(), 
                                      path_from_neighbor.begin()+1, 
                                      path_from_neighbor.end());
                    break;
                }
            }
        }
    }
    
    if (path_segment.empty()) {
        Serial.println("No valid path found! Waiting...");
        setMotorSpeeds(0, 0);
        delay(5000); // Wait 5 seconds and try again
        planMission();
        return;
    }
    
    full_path = path_segment;
    current_path_index = 0;
    
    Serial.print("New path: ");
    for (const auto& node : full_path) {
        Serial.print(node.c_str()); Serial.print(" ");
    }
    Serial.println();
}

std::vector<String> dijkstra(String start, String goal) {
    // Priority queue: (total_distance, current_node)
    std::priority_queue<std::pair<int, String>, 
                       std::vector<std::pair<int, String>>, 
                       std::greater<std::pair<int, String>>> pq;
    std::map<String, bool> visited;
    std::map<String, String> previous;
    std::map<String, int> distances;
    
    // Initialize distances
    for (auto& node : graph) {
        distances[node.first] = INT_MAX;
    }
    distances[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        String current = pq.top().second;
        pq.pop();
        
        if (current == goal) break;
        if (visited[current]) continue;
        
        visited[current] = true;
        
        for (auto& neighbor : graph[current]) {
            String next = std::get<0>(neighbor);
            int weight = std::get<1>(neighbor);
            
            if (distances[next] > distances[current] + weight) {
                distances[next] = distances[current] + weight;
                previous[next] = current;
                pq.push({distances[next], next});
            }
        }
    }
    
    // Reconstruct path
    std::vector<String> path;
    for (String at = goal; at != start; at = previous[at]) {
        path.insert(path.begin(), at);
    }
    path.insert(path.begin(), start);
    
    return path;
}

bool isApproachingTargetBox() {
    return (processing_targets && 
            !full_path.empty() && 
            current_path_index + 1 < full_path.size() &&
            full_path[current_path_index + 1] == TARGET_NODES[boxes_collected]);
}

// ==================== HELPER FUNCTIONS ====================

void configureTOF(VL6180X& sensor, uint8_t address) {
    sensor.init();
    sensor.configureDefault();
    sensor.setAddress(address);
    sensor.setTimeout(500);
}

int getCurrentFaceTOFDistance() {
    switch(current_face) {
        case 0: return tof1.readRangeSingleMillimeters();
        case 1: return tof2.readRangeSingleMillimeters();
        case 2: return tof3.readRangeSingleMillimeters();
        case 3: return tof4.readRangeSingleMillimeters();
        default: return -1;
    }
}

void activateElectromagnet(bool on) {
    switch(current_face) {
        case 0: digitalWrite(EMAG1, on ? HIGH : LOW); break;
        case 1: digitalWrite(EMAG2, on ? HIGH : LOW); break;
        case 2: digitalWrite(EMAG3, on ? HIGH : LOW); break;
        case 3: digitalWrite(EMAG4, on ? HIGH : LOW); break;
    }
}

void followLine(bool line_left, bool line_center, bool line_right) {
    float error = 0;
    
    if (line_left && !line_center && !line_right) {
        error = -1;
    } 
    else if (line_right && !line_center && !line_left) {
        error = 1;
    } 
    else if (line_center && !line_left && !line_right) {
        error = 0;
    } 
    else if (line_left && line_center && !line_right) {
        error = -0.5;
    } 
    else if (line_right && line_center && !line_left) {
        error = 0.5;
    } 
    else {
        error = 0;  
    }
    
    float correction = pid_calc(error);
    setMotorSpeeds(LINE_FOLLOW_SPEED - correction, LINE_FOLLOW_SPEED + correction);
}

float pid_calc(float error) {
    integral += error;
    float derivative = error - prev_err;
    prev_err = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

void avoidObstacle() {
    // Simplified version since we now have handleObstacleAvoidance
    Serial.println("Small obstacle detected, quick avoidance...");
    
    // Back up slightly
    setMotorSpeeds(-0.3 * MAX_SPEED, -0.3 * MAX_SPEED);
    delay(300);
    
    // Turn slightly
    setMotorSpeeds(0.3 * MAX_SPEED, -0.3 * MAX_SPEED);
    delay(200);
    
    // Continue forward
    setMotorSpeeds(0.5 * MAX_SPEED, 0.5 * MAX_SPEED);
    delay(300);
}

void setMotorSpeeds(float left, float right) {
    // Constrain speeds to valid range
    left = constrain(left, -MAX_SPEED, MAX_SPEED);
    right = constrain(right, -MAX_SPEED, MAX_SPEED);
    
    // Set left motor direction and speed
    digitalWrite(LEFT_MOTOR_DIR, left > 0 ? HIGH : LOW);
    analogWrite(LEFT_MOTOR_PWM, abs(left) * 255 / MAX_SPEED);
    
    // Set right motor direction and speed
    digitalWrite(RIGHT_MOTOR_DIR, right > 0 ? HIGH : LOW);
    analogWrite(RIGHT_MOTOR_PWM, abs(right) * 255 / MAX_SPEED);
}

void printMissionStatus() {
    Serial.println("\n=== MISSION STATUS ===");
    if (processing_targets) {
        Serial.printf("Phase: COLLECTING | Boxes: %d/4\n", boxes_collected);
    } 
    else {
        Serial.printf("Phase: DELIVERING | Boxes: %d/4\n", boxes_delivered);
    }
    
    Serial.printf("Current face: %d | Loaded faces: ", current_face + 1);
    for (int i = 0; i < 4; i++) {
        Serial.print(boxes_loaded[i] ? "1 " : "0 ");
    }
    Serial.printf("\nCurrent node: %s\n", node_tracking->current_node.c_str());
    Serial.println("=====================");
}
