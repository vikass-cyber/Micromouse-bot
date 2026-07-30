#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. MAZE SOLVER DEFINITIONS & BITMASKS
// ============================================================================

#define MAZE_SIZE 16
#define MAX_QUEUE_SIZE (MAZE_SIZE * MAZE_SIZE)

// Bitmask Flags for Wall and Visited Memory
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08
#define VISITED 0x10

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum MotionCmd { CMD_FORWARD_1, CMD_TURN_LEFT, CMD_TURN_RIGHT, CMD_TURN_180, CMD_STOP };

struct Point {
    uint8_t x, y;
};

// ============================================================================
// 2. MAZE SOLVER CLASS DECLARATION
// ============================================================================

class MazeSolver {
public:
    MazeSolver();
    void init();
    
    // Maze Bitmask Manipulators
    void setWall(uint8_t x, uint8_t y, Direction dir);
    bool hasWall(uint8_t x, uint8_t y, Direction dir);
    void setVisited(uint8_t x, uint8_t y);
    bool isVisited(uint8_t x, uint8_t y);
    
    // Dynamic Floodfill Engine
    void updateFloodFill(uint8_t targetX1, uint8_t targetY1, uint8_t targetX2 = 255, uint8_t targetY2 = 255);
    uint8_t getDistance(uint8_t x, uint8_t y);
    
    // Navigation Decisions
    Direction getBestNeighborDir(uint8_t currentX, uint8_t currentY, Direction currentHeading);
    MotionCmd getNextMotionCommand(uint8_t currentX, uint8_t currentY, Direction currentHeading);

private:
    uint8_t cells[MAZE_SIZE][MAZE_SIZE];
    uint8_t dist[MAZE_SIZE][MAZE_SIZE];
    
    // Queue Implementation for BFS Floodfill
    Point queue[MAX_QUEUE_SIZE];
    int qHead, qTail;
    
    void pushQueue(uint8_t x, uint8_t y);
    Point popQueue();
    bool isQueueEmpty();
    void clearQueue();
};

// ============================================================================
// 3. MAZE SOLVER CLASS IMPLEMENTATION
// ============================================================================

MazeSolver::MazeSolver() {
    init();
}

void MazeSolver::init() {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            cells[x][y] = 0;
            dist[x][y] = 255; // Set distance map to unvisited maximum
            
            // Set perimeter boundary walls
            if (y == MAZE_SIZE - 1) cells[x][y] |= WALL_N;
            if (x == MAZE_SIZE - 1) cells[x][y] |= WALL_E;
            if (y == 0)             cells[x][y] |= WALL_S;
            if (x == 0)             cells[x][y] |= WALL_W;
        }
    }
}

void MazeSolver::setWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return;
    
    switch (dir) {
        case NORTH:
            cells[x][y] |= WALL_N;
            if (y < MAZE_SIZE - 1) cells[x][y + 1] |= WALL_S;
            break;
        case EAST:
            cells[x][y] |= WALL_E;
            if (x < MAZE_SIZE - 1) cells[x + 1][y] |= WALL_W;
            break;
        case SOUTH:
            cells[x][y] |= WALL_S;
            if (y > 0) cells[x][y - 1] |= WALL_N;
            break;
        case WEST:
            cells[x][y] |= WALL_W;
            if (x > 0) cells[x - 1][y] |= WALL_E;
            break;
    }
}

bool MazeSolver::hasWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return true;
    switch (dir) {
        case NORTH: return (cells[x][y] & WALL_N) != 0;
        case EAST:  return (cells[x][y] & WALL_E) != 0;
        case SOUTH: return (cells[x][y] & WALL_S) != 0;
        case WEST:  return (cells[x][y] & WALL_W) != 0;
    }
    return true;
}

void MazeSolver::setVisited(uint8_t x, uint8_t y) {
    if (x < MAZE_SIZE && y < MAZE_SIZE) cells[x][y] |= VISITED;
}

bool MazeSolver::isVisited(uint8_t x, uint8_t y) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return false;
    return (cells[x][y] & VISITED) != 0;
}

uint8_t MazeSolver::getDistance(uint8_t x, uint8_t y) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return 255;
    return dist[x][y];
}

void MazeSolver::pushQueue(uint8_t x, uint8_t y) {
    if (qTail < MAX_QUEUE_SIZE) {
        queue[qTail++] = {x, y};
    }
}

Point MazeSolver::popQueue() {
    return queue[qHead++];
}

bool MazeSolver::isQueueEmpty() {
    return qHead >= qTail;
}

void MazeSolver::clearQueue() {
    qHead = 0;
    qTail = 0;
}

void MazeSolver::updateFloodFill(uint8_t targetX1, uint8_t targetY1, uint8_t targetX2, uint8_t targetY2) {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            dist[x][y] = 255;
        }
    }

    clearQueue();

    // Push primary target
    dist[targetX1][targetY1] = 0;
    pushQueue(targetX1, targetY1);
    
    // Push secondary target area if 2x2 goal center is used
    if (targetX2 < MAZE_SIZE && targetY2 < MAZE_SIZE) {
        dist[targetX2][targetY1] = 0; pushQueue(targetX2, targetY1);
        dist[targetX1][targetY2] = 0; pushQueue(targetX1, targetY2);
        dist[targetX2][targetY2] = 0; pushQueue(targetX2, targetY2);
    }

    while (!isQueueEmpty()) {
        Point curr = popQueue();
        uint8_t currentDist = dist[curr.x][curr.y];

        // Check North Neighbor
        if (!hasWall(curr.x, curr.y, NORTH) && curr.y < MAZE_SIZE - 1) {
            if (dist[curr.x][curr.y + 1] == 255) {
                dist[curr.x][curr.y + 1] = currentDist + 1;
                pushQueue(curr.x, curr.y + 1);
            }
        }
        // Check East Neighbor
        if (!hasWall(curr.x, curr.y, EAST) && curr.x < MAZE_SIZE - 1) {
            if (dist[curr.x + 1][curr.y] == 255) {
                dist[curr.x + 1][curr.y] = currentDist + 1;
                pushQueue(curr.x + 1, curr.y);
            }
        }
        // Check South Neighbor
        if (!hasWall(curr.x, curr.y, SOUTH) && curr.y > 0) {
            if (dist[curr.x][curr.y - 1] == 255) {
                dist[curr.x][curr.y - 1] = currentDist + 1;
                pushQueue(curr.x, curr.y - 1);
            }
        }
        // Check West Neighbor
        if (!hasWall(curr.x, curr.y, WEST) && curr.x > 0) {
            if (dist[curr.x - 1][curr.y] == 255) {
                dist[curr.x - 1][curr.y] = currentDist + 1;
                pushQueue(curr.x - 1, curr.y);
            }
        }
    }
}

Direction MazeSolver::getBestNeighborDir(uint8_t x, uint8_t y, Direction currentHeading) {
    uint8_t minDistance = 255;
    Direction bestDir = currentHeading;

    // Preference priority: Current Heading -> Right -> Left -> Reverse
    Direction dirs[4] = { currentHeading, 
                          (Direction)((currentHeading + 1) % 4), 
                          (Direction)((currentHeading + 3) % 4), 
                          (Direction)((currentHeading + 2) % 4) };

    for (int i = 0; i < 4; i++) {
        Direction d = dirs[i];
        if (!hasWall(x, y, d)) {
            uint8_t nx = x, ny = y;
            if (d == NORTH) ny++;
            else if (d == EAST) nx++;
            else if (d == SOUTH) ny--;
            else if (d == WEST) nx--;

            if (dist[nx][ny] < minDistance) {
                minDistance = dist[nx][ny];
                bestDir = d;
            }
        }
    }
    return bestDir;
}

MotionCmd MazeSolver::getNextMotionCommand(uint8_t x, uint8_t y, Direction currentHeading) {
    Direction targetDir = getBestNeighborDir(x, y, currentHeading);

    if (targetDir == currentHeading) {
        return CMD_FORWARD_1;
    } else if (targetDir == (currentHeading + 1) % 4) {
        return CMD_TURN_RIGHT;
    } else if (targetDir == (currentHeading + 3) % 4) {
        return CMD_TURN_LEFT;
    } else {
        return CMD_TURN_180;
    }
}

// ============================================================================
// 4. HARDWARE CONFIGURATIONS & MAIN SYSTEM INSTANCES
// ============================================================================

#define PIN_IR_LEFT      36
#define PIN_IR_FRONT     39
#define PIN_IR_RIGHT     35
#define PIN_BUTTON       0
#define PIN_BUZZER       18

#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MazeSolver solver;

// Pose Tracker
uint8_t posX = 0;
uint8_t posY = 0;
Direction heading = NORTH;

// Wall Calibration Thresholds
const int THRESH_LEFT = 800;
const int THRESH_FRONT = 1200;
const int THRESH_RIGHT = 800;

// ============================================================================
// 5. DIAGNOSTICS & SYSTEM HELPERS
// ============================================================================

void renderDashboard(const char* actionState) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.printf("PHASE 3: %s\n", actionState);
    display.printf("POS  : (%d, %d)\n", posX, posY);
    display.printf("HEAD : %s\n", heading == NORTH ? "NORTH" : heading == EAST ? "EAST" : heading == SOUTH ? "SOUTH" : "WEST");
    display.printf("DIST : %d\n", solver.getDistance(posX, posY));
    display.display();
    
    Serial.printf("[P3 NAV] Pos:(%d,%d) | Head:%d | Dist:%d | Cmd:%s\n", 
                  posX, posY, heading, solver.getDistance(posX, posY), actionState);
}

void scanWallsAndUpdateMap() {
    int rawL = analogRead(PIN_IR_LEFT);
    int rawF = analogRead(PIN_IR_FRONT);
    int rawR = analogRead(PIN_IR_RIGHT);

    bool wallL = (rawL > THRESH_LEFT);
    bool wallF = (rawF > THRESH_FRONT);
    bool wallR = (rawR > THRESH_RIGHT);

    // Map local robot-centric walls to global cardinal directions
    if (wallF) solver.setWall(posX, posY, heading);
    if (wallR) solver.setWall(posX, posY, (Direction)((heading + 1) % 4));
    if (wallL) solver.setWall(posX, posY, (Direction)((heading + 3) % 4));

    solver.setVisited(posX, posY);
}

void updateCoordinates() {
    if (heading == NORTH) posY++;
    else if (heading == EAST) posX++;
    else if (heading == SOUTH) posY--;
    else if (heading == WEST) posX--;
}

// ============================================================================
// 6. SETUP & MAIN NAVIGATION LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    Wire.begin(21, 22);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    solver.init();
    
    // Set competition center destination (7,7 to 8,8)
    solver.updateFloodFill(7, 7, 8, 8);

    renderDashboard("STANDBY");

    while (digitalRead(PIN_BUTTON) == HIGH) {
        delay(10);
    }
    
    tone(PIN_BUZZER, 2000, 100);
    delay(500);
}

void loop() {
    // Check Goal Condition
    if (solver.getDistance(posX, posY) == 0) {
        tone(PIN_BUZZER, 3000, 1000);
        while (true) {
            renderDashboard("GOAL REACHED!");
            delay(500);
        }
    }

    // Step 1: Scan current cell
    scanWallsAndUpdateMap();

    // Step 2: Compute shortest dynamic paths
    solver.updateFloodFill(7, 7, 8, 8);

    // Step 3: Determine optimal move
    MotionCmd cmd = solver.getNextMotionCommand(posX, posY, heading);

    // Step 4: Update heading and location states
    switch (cmd) {
        case CMD_FORWARD_1:
            renderDashboard("MOVE FWD");
            updateCoordinates();
            break;

        case CMD_TURN_RIGHT:
            renderDashboard("TURN RIGHT");
            heading = (Direction)((heading + 1) % 4);
            break;

        case CMD_TURN_LEFT:
            renderDashboard("TURN LEFT");
            heading = (Direction)((heading + 3) % 4);
            break;

        case CMD_TURN_180:
            renderDashboard("TURN 180");
            heading = (Direction)((heading + 2) % 4);
            break;

        case CMD_STOP:
        default:
            renderDashboard("STOPPED");
            while (true);
    }
    
    delay(300); // Execution delay for step-by-step validation
}