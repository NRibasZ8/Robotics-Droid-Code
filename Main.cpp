#include <Wire.h>
#include <VL6180X.h>
#include <Arduino.h>
#include <vector>
#include <queue>
#include <map>

// TCRT5000 Sensor Pins
const int S1 = 13, S2 = 12, S3 = 14, S4 = 27, S5 = 26;
const int CLP = 25;    // Limit switch
const int NEAR = 33;   // Proximity sensor

// Motor control pins
const int LEFT_MOTOR_PWM = 32;
const int LEFT_MOTOR_DIR = 23;
const int RIGHT_MOTOR_PWM = 18;
const int RIGHT_MOTOR_DIR = 19;

// VL6180X Sensor
VL6180X vl6180;

// Navigation constants
const float BASE_SPEED = 0.6;  // Base speed (0 to 1)
const float KP = 0.15;         // Proportional constant for line following
const unsigned long NODE_DETECT_DELAY = 500; // ms to wait at node

// PID constants
const float Kp = 0.75;
const float Ki = 0.001;
const float Kd = 0.08;
float integral = 0;
float prev_err = 0;

// Parameters for movement
const int TURN_90 = 44;        // Step count for a 90° turn
const int TURN_180 = 88;        // Step count for a 180° turn
const int TURN_INTERSECTION = 10; // Steps to exit a intersection
const float MAX_SPEED = 6.28;   // Max wheel velocity (2π radians)
const float TURN_SPEED = 0.25 * MAX_SPEED; // Angular velocity while turning
const float LINE_FOLLOW_SPEED = 0.3 * MAX_SPEED; // Angular velocity while following a line

// Override turn distances for tricky intersections/sensor misalignment
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

// Robot state machine states
enum State {
    FOLLOW_LINE,
    TURN_PREP,
    IN_TURN,
    SEARCH,
    INTERSECTION,
    COMPLETED
};

// Node graph: all nodes have neighbouring nodes with (Node name, Distance, Direction)
// Directions: 0 = North, 1 = East, 2 = South, 3 = West
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

// Keeps track of the EPUCK's current position as a node from the graph
class TrackNode {
public:
    String current_node;
    int direction;
    
    TrackNode(std::map<String, std::vector<std::tuple<String, int, int>>> graph, String start_node, int start_direction) {
        this->graph = graph;
        this->current_node = start_node;
        this->direction = start_direction;
    }
    
    std::pair<int, int> get_turn_to(String next_node) {
        for (auto& neighbour : graph[current_node]) {
            if (std::get<0>(neighbour) == next_node) {
                int relative_direction = (std::get<2>(neighbour) - direction) % 4;
                Serial.printf("Current node: %s, Next planned node: %s, direction: %d\n", current_node.c_str(), next_node.c_str(), direction);
                return {relative_direction, std::get<2>(neighbour)};
            }
        }
        Serial.printf("No path found from %s to %s\n", current_node.c_str(), next_node.c_str());
        return {0, 0}; // Default return, should handle error properly
    }
    
    int advance(String next_node) {
        auto turn_info = get_turn_to(next_node);
        int relative_direction = turn_info.first;
        int absolute_direction = turn_info.second;
        Serial.printf("Moving from %s to %s. New direction: %d\n", current_node.c_str(), next_node.c_str(), absolute_direction);
        current_node = next_node;
        direction = absolute_direction;
        return relative_direction;
    }

private:
    std::map<String, std::vector<std::tuple<String, int, int>>> graph;
};

// Global variables
State state = FOLLOW_LINE;
int turn_counter = 0;
int turn_direction = 0;
int turn_steps_target = 0;
String next_node = "";
bool turn_lost_line = false;
int intersection_cooldown = 0;
const int intersection_cooldown_steps = 32;
bool past_intersection = false;
String START_NODE = "7A";  // Start node/initial position
String GOAL_NODE = "3B";   // Robot's goal node
std::vector<String> PATH;
int path_index = 0;
TrackNode* node_tracking;

void setup() {
    Serial.begin(115200);
    
    // Initialize TCRT5000 pins
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
    
    // Initialize I2C for VL6180X
    Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
    
    // Wait for sensor to boot up
    delay(100);
    
    // Initialize VL6180X
    vl6180.init();
    vl6180.configureDefault();
    vl6180.setTimeout(500);
    
    // Initialize path (in a real implementation, you'd get this from path planning)
    PATH = {"7A", "6A", "6B", "6C", "6D", "6E", "5A", "4B", "3B"};
    node_tracking = new TrackNode(graph, PATH[0], 2); // Start facing South
    
    Serial.println("\nAll systems ready!");
}

void loop() {
    // Read line sensors (1 = on line, 0 = off line)
    bool s1 = !digitalRead(S1);
    bool s2 = !digitalRead(S2);
    bool s3 = !digitalRead(S3);
    bool s4 = !digitalRead(S4);
    bool s5 = !digitalRead(S5);
    
    bool line_left = s1;
    bool line_center = s3;
    bool line_right = s5;
    bool intersection_now = (s1 + s3 + s5) > 1;
    
    // Update intersection marker to prevent re-triggering
    if (!intersection_now) {
        past_intersection = true;
    }
    
    // State machine implementation
    switch(state) {
        case FOLLOW_LINE:
            followLine(line_left, line_center, line_right);
            
            // Intersection detection and path deciding
            if (intersection_cooldown == 0 && intersection_now && past_intersection && path_index + 1 < PATH.size()) {
                next_node = PATH[path_index + 1];
                auto turn_info = node_tracking->get_turn_to(next_node);
                turn_direction = turn_info.first;
                past_intersection = false;
                if (turn_direction != 0) {
                    state = TURN_PREP;
                    turn_counter = 0;
                    turn_lost_line = false;
                } else {
                    state = INTERSECTION;
                    turn_counter = 0;
                }
            }
            break;
            
        case TURN_PREP:
            setMotorSpeeds(LINE_FOLLOW_SPEED, LINE_FOLLOW_SPEED);
            turn_counter++;
            if (turn_counter >= 4) {
                state = IN_TURN;
                turn_counter = 0;
                // Use turn correction if required
                auto key = std::make_pair(node_tracking->current_node, next_node);
                if (ADJUSTED_TURNS.find(key) != ADJUSTED_TURNS.end()) {
                    turn_steps_target = ADJUSTED_TURNS[key];
                } else {
                    turn_steps_target = (turn_direction == 1 || turn_direction == 3) ? TURN_90 : TURN_180;
                }
            }
            break;
            
        case IN_TURN:
            if (turn_direction == 1) {
                setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
            } else if (turn_direction == 3) {
                setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
            } else if (turn_direction == 2) {
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
                path_index++;
                intersection_cooldown = intersection_cooldown_steps;
                past_intersection = false;
                state = (path_index == PATH.size() - 1) ? COMPLETED : FOLLOW_LINE;
                turn_counter = 0;
            }
            break;
            
        case COMPLETED:
            setMotorSpeeds(0, 0);
            while(1) { delay(1000); } // Stop forever
            break;
    }
    
    if (intersection_cooldown > 0) {
        intersection_cooldown--;
    }
    
    // Check for obstacles
    float distance = vl6180.readRangeSingleMillimeters();
    if (!vl6180.timeoutOccurred() && distance < 150) { // 150mm = 15cm
        avoidObstacle();
    }
    
    delay(10); // Small delay for stability
}

float pid_calc(float error) {
    integral += error;
    float derivative = error - prev_err;
    prev_err = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

void followLine(bool line_left, bool line_center, bool line_right) {
    float error = 0;
    
    if (line_left && !line_center && !line_right) {
        error = -1;
    } else if (line_right && !line_center && !line_left) {
        error = 1;
    } else if (line_center && !line_left && !line_right) {
        error = 0;
    } else if (line_left && line_center && !line_right) {
        error = -0.5;
    } else if (line_right && line_center && !line_left) {
        error = 0.5;
    } else {
        error = 0;  
    }
    
    float correction = pid_calc(error);
    setMotorSpeeds(LINE_FOLLOW_SPEED - correction, LINE_FOLLOW_SPEED + correction);
}

void avoidObstacle() {
    Serial.println("Obstacle detected! Avoiding...");
    
    // Back up slightly
    setMotorSpeeds(-0.3 * MAX_SPEED, -0.3 * MAX_SPEED);
    delay(500);
    
    // Turn right
    setMotorSpeeds(0.3 * MAX_SPEED, -0.3 * MAX_SPEED);
    delay(500);
    
    // Continue forward
    setMotorSpeeds(0.5 * MAX_SPEED, 0.5 * MAX_SPEED);
    delay(500);
    
    // Turn left to realign with line
    setMotorSpeeds(-0.3 * MAX_SPEED, 0.3 * MAX_SPEED);
    delay(500);
    
    // Resume line following
}

void setMotorSpeeds(float left, float right) {
    // Set left motor direction and speed
    digitalWrite(LEFT_MOTOR_DIR, left > 0 ? HIGH : LOW);
    analogWrite(LEFT_MOTOR_PWM, abs(left) * 255 / MAX_SPEED);
    
    // Set right motor direction and speed
    digitalWrite(RIGHT_MOTOR_DIR, right > 0 ? HIGH : LOW);
    analogWrite(RIGHT_MOTOR_PWM, abs(right) * 255 / MAX_SPEED);
}
