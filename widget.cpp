#include "widget.h"
#define WIDTH 600
#define HEIGHT 400

#include <queue>
#include <unordered_set>
#include "setseo.h"
#include "setrobot.h"
#include "setmoveo.h"

Widget::Widget(QWidget *parent, int height, int width)
    : QWidget(parent), gridHeight(height), gridWidth(width)
{

    initializeGrid(gridHeight, gridWidth);

    // Set window title.
    this->setWindowTitle("Path Simulation");

    // Set window size.
    // this->resize(WIDTH, HEIGHT);
    // this->setMinimumSize(WIDTH, HEIGHT);
    this->setFixedSize(gridWidth * 20 + 400, gridHeight * 20);

    // Set layout.
    QVBoxLayout* bigLayout = new QVBoxLayout(this);
    bigLayout -> setContentsMargins(gridWidth * 20 + 10, 0, 10, 40);

    QHBoxLayout* topLayout = new QHBoxLayout();
    QPushButton* setStartEndObstacles = new QPushButton("1.Basic operation");
    QPushButton* setRobotAttribute = new QPushButton("2.Robot setting");
    QPushButton* setMoveableObs = new QPushButton("3.Mobile obstacles");
    topLayout -> addWidget(setStartEndObstacles);
    topLayout -> addWidget(setRobotAttribute);
    topLayout -> addWidget(setMoveableObs);
    setStartEndObstacles -> setFixedSize(130, 30);
    setRobotAttribute -> setFixedSize(130, 30);
    setMoveableObs -> setFixedSize(130, 30);
    connect(setStartEndObstacles, &QPushButton::clicked, this, &Widget::showSEOWidget);
    connect(setRobotAttribute, &QPushButton::clicked, this, &Widget::showRobotWidget);
    connect(setMoveableObs, &QPushButton::clicked, this, &Widget::showMoveOWidget);


    pStack = new QStackedWidget();
    SetSEO* setSEO = new SetSEO();
    SetRobot* setRobot = new SetRobot();
    SetMoveO* setMoveO = new SetMoveO();
    pStack -> addWidget(setSEO);
    pStack -> addWidget(setRobot);
    pStack -> addWidget(setMoveO);

    // Add push button to search path.
    QPushButton* searchPathButton = new QPushButton("Search Path", this);
    connect(searchPathButton, &QPushButton::clicked, this, &Widget::searchPathButtonClicked);

    bigLayout -> addLayout(topLayout);
    bigLayout -> addWidget(pStack);
    // bigLayout -> addWidget(searchPathButton);
    searchPathButton -> setGeometry(gridWidth * 20 + 50, gridHeight * 20 - 40, 300, 30);

}


// Initialize.
void Widget::initializeGrid(int height, int width){
    grid.resize(width);
    for(int i = 0; i < width; i++){
        grid[i].resize(height);
        for(int j = 0; j < height; j++){
            grid[i][j] = Node(i, j);
        }
    }
}

// The function of receiving paint events passed in event.
void Widget::paintEvent(QPaintEvent* event){
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    drawGrid(painter);
    if(robotInitialized){
        drawRobot(painter);
    }
}


// Make initialize grid.
void Widget::drawGrid(QPainter& painter){
    painter.setPen(Qt::black);
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            // Set the position and size of every grid
            QRect rect(i * 20, j * 20, 20, 20);
            // Set color of different grid
            if(grid[i][j].isObstacle){
                painter.fillRect(rect, Qt::black);
            }
            else if(&grid[i][j] == startNode){
                painter.fillRect(rect, Qt::green);
            }
            else if(endNode.contains(&grid[i][j])){
                painter.fillRect(rect, Qt::red);
            }
            painter.drawRect(rect);
        }
    }
}

// The function of mouse click event.
void Widget::mousePressEvent(QMouseEvent* event){
    if(event -> pos().x() > gridWidth * 20 || event -> pos().y() > gridHeight * 20){
        return;
    }
    int x = event -> pos().x() / 20;
    int y = event -> pos().y() / 20;
    switch(mode){
    case 0:
        // Use leftbutton set obstacle.
        if(event -> button() == Qt::LeftButton){
            grid[x][y].isObstacle = !grid[x][y].isObstacle;
            if(obstacles.find({grid[x][y].x * 20 + 10, grid[x][y].y * 20 + 10}) == obstacles.end()){
                obstacles.insert({grid[x][y].x * 20 + 10, grid[x][y].y * 20 + 10});
            }
            else{
                obstacles.remove({grid[x][y].x * 20 + 10, grid[x][y].y * 20 + 10});
            }
        }
        // Use rightbutton set start/end node.
        else if(event -> button() == Qt::RightButton){

            if(!startNode){
                startNode = &grid[x][y];
            }
            else{
                endNode.push_back(&grid[x][y]);
            }
        }
        break;
    case 1:
        break;
    case 2:
        break;
    }


    update();
}

// Use Euclidean distance as the heuristic function.
double Widget::heuristic(const Node* node, const Node* endNode){
    return sqrt((node -> x - endNode -> x) * (node -> x - endNode -> x) + (node -> y - endNode -> y) * (node -> y - endNode -> y));
}

// Reset g/h/f/parent.
void Widget::reset(){
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[0].size(); j++){
            grid[i][j].g = grid[i][j].f = INT_MAX;
            grid[i][j].h = 0;
            grid[i][j].parent = nullptr;
        }
    }
}

// Get Theta* path.
QVector<Node*> Widget::findPath(Node* node1, Node* node2){
    if(!node1 || !node2){
        return {};
    }
    reset();
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openQueue;
    std::unordered_set<Node*> openSet;
    std::unordered_set<Node*> closeSet;
    // Initialize.
    node1 -> g = 0;
    node1 -> h = node1 -> f = heuristic(node1, node2);
    openQueue.push(node1);
    openSet.insert(node1);
    // Theta*.
    while(!openSet.empty()){
        Node* current = openQueue.top();
        openQueue.pop();
        openSet.erase(current);
        // If get endNode, return path.
        if(current == node2){
            QVector<Node*> path;
            while(current){
                path.push_back(current);
                current = current -> parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closeSet.insert(current);
        static const int dxdy[4][2] = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
        for(int i = 0; i < 4; i++){
            int newX = current -> x + dxdy[i][0];
            int newY = current -> y + dxdy[i][1];
            // If out or isObstacle.
            if(newX < 0 || newX >= grid.size() || newY < 0 || newY >= grid[0].size() || grid[newX][newY].isObstacle){
                continue;
            }

            Node* nextNode = &grid[newX][newY];
            // If already in closeSet.
            if(closeSet.find(nextNode) != closeSet.end()){
                continue;
            }

            // Calculate g/h/f.
            int newG = current -> g + 1;
            // If newG >= nextNode -> g , just continue because we have shorter path to the nextNode.
            if(newG < nextNode -> g){
                nextNode -> parent = current;
                nextNode -> g = newG;
                nextNode -> h = heuristic(nextNode, node2);
                nextNode -> f = nextNode -> g + nextNode -> h;

                // Update to Theta*
                if(hitTestWithLine(current -> parent, nextNode)){
                    nextNode -> parent = current -> parent;
                    nextNode -> g = current -> parent -> g + heuristic(current -> parent, nextNode);
                    nextNode -> f = nextNode -> g + nextNode -> h;
                }


                if(openSet.find(nextNode) == openSet.end()){
                    openQueue.push(nextNode);
                    openSet.insert(nextNode);
                }
            }
        }
    }
    inaccessible = true;
    return {};
}

// Check line for Theta*.
bool Widget::hitTestWithLine(Node* parent, Node* nextNode){
    if(!parent || !nextNode){
        return false;
    }
    double x0 = parent -> x;
    double y0 = parent -> y;
    double x1 = nextNode -> x;
    double y1 = nextNode -> y;
    // (y - y0) = k * (x - x0)
    if(x1 == x0){
        if(y0 > y1){
            std::swap(y0 , y1);
        }
        x0 = x0 * 20 + 10;
        for(double dy = y0 + 1; dy < y1; dy += 1){
            double cury = dy * 20 + 10;
            for(std::pair<double, double> node : obstacles){
                if(node.first != x0){
                    continue;
                }
                if(cury == node.second){
                    return false;
                }
            }
        }
    }
    else if(y1 == y0){
        if(x0 > x1){
            std::swap(x0, x1);
        }
        y0 = y0 * 20 + 10;
        for(double dx = x0 + 1; dx < x1; dx += 1){
            double curx = dx * 20 + 10;
            for(std::pair<double, double> node : obstacles){
                if(node.second != y0){
                    continue;
                }
                if(curx == node.first){
                    return false;
                }
            }
        }
    }
    else{
        double k = (y1 - y0) / (x1 - x0);
        x0 = x0 * 20 + 10;
        y0 = y0 * 20 + 10;
        x1 = x1 * 20 + 10;
        y1 = y1 * 20 + 10;
        for(std::pair<double, double> node : obstacles){
            if(node.first + 10 < std::min(x0, x1) || node.first - 10 > std::max(x0, x1) || node.second + 10 < std::min(y0, y1) || node.second - 10 > std::max(y0, y1)){
                continue;
            }
            double windowTop = node.second - 10;
            double windowBottom = node.second + 10;
            double windowLeft = node.first - 10;
            double windowRight = node.first + 10;
            // y = k * (x - x0) + y0;
            double leftPoint = k * (windowLeft - x0) + y0;
            if(leftPoint <= windowBottom && leftPoint >= windowTop){
                return false;
            }
            double rightPoint = k * (windowRight - x0) + y0;
            if(rightPoint <= windowBottom && rightPoint >= windowTop){
                return false;
            }
            // x = (y - y0) / k + x0;
            double topPoint = (windowTop - y0) / k + x0;
            if(topPoint <= windowRight && topPoint >= windowLeft){
                return false;
            }
            double bottomPoint = (windowBottom - y0) / k + x0;
            if(bottomPoint <= windowRight && bottomPoint >= windowLeft){
                return false;
            }
        }
    }
    return true;
}

// Calculate path size.
int Widget::calculatePath(int start, int end) const{
    if(start > end){
        std::swap(start, end);
    }
    return lengthTable[start][end - start - 1];
}

// Get all possible path using backtracking algorithm and choose the shortest path.
void Widget::getFullArrangement(QVector<int>& nums, QVector<bool>& used, QVector<int>& path){
    if(path.size() == nums.size() + 1){
        int curPathSize = 0;
        for(int i = 0; i < nums.size(); i++){
            curPathSize += calculatePath(path[i], path[i + 1]);
        }
        if(curPathSize < bestPathSize){
            bestPathSize = curPathSize;
            bestPath = path;
        }
    }

    for(int i = 0; i < nums.size(); i++){
        if(used[i]){
            continue;
        }
        path.push_back(nums[i]);
        used[i] = true;
        getFullArrangement(nums, used, path);
        used[i] = false;
        path.pop_back();
    }

}

// If click searchPathButton.
void Widget::searchPathButtonClicked(){
    // Get pathTable.
    nodes.push_back(startNode);
    for(int i = 0; i < endNode.size(); i++){
        nodes.push_back(endNode[i]);
    }
    pathTable.resize(nodes.size() - 1);
    lengthTable.resize(nodes.size() - 1);
    for(int i = 0; i < nodes.size() - 1; i++){
        pathTable[i].resize(nodes.size() - 1 - i);
        lengthTable[i].resize(nodes.size() - 1 - i);
        for(int j = 0; j < nodes.size() - 1 - i; j++){
            pathTable[i][j] = findPath(nodes[i], nodes[i + j + 1]);
            lengthTable[i][j] = pathTable[i][j].back()->g;
        }
    }

    if(inaccessible){
        qDebug() << "Error:Can't arrive at all end nodes";
        return;
    }

    // Get full arrangement.
    QVector<int> nums;
    QVector<bool> used(endNode.size(), false);
    for(int i = 0; i < endNode.size(); i++){
        nums.push_back(i + 1);
    }
    QVector<int> path = {0};
    getFullArrangement(nums, used, path);
    // Output best path.
    result.push_back(startNode);
    for(int i = 0; i < bestPath.size() - 1; i++){
        int start = bestPath[i];
        int end = bestPath[i + 1];
        if(start > end){
            std::swap(start, end);
            pathTable[start][end - start - 1];
            std::reverse(pathTable[start][end - start - 1].begin(), pathTable[start][end - start - 1].end());
        }
        QVector<Node*> curPath = pathTable[start][end - start - 1];
        for(int j = 1; j < curPath.size(); j++){
            result.push_back(curPath[j]);
        }
    }

    if(!result.empty()){
        for(Node* node : result){
            qDebug() << "(" << node -> x << ", " << node -> y << ")";
        }
        qDebug() << "length: " << result.size() - 1;
    }

    /*
    for(int i = 0; i < bestPath.size(); i++){
        qDebug() << bestPath[i];
    }
    */

    /*
    QVector<Node*> path = findPath(startNode, endNode[0]);
    if(!path.empty()){
        for(Node* node : path){
            qDebug() << "(" << node -> x << ", " << node -> y << ")";
        }
        qDebug() << "length: " << path.size();
    }
    else{
        qDebug() << "Not find path";
    }
    */

    DWAStart();

}

// Convert point coordinates to pixel coordinates.
void Widget::CoordinateTransformation(){
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[0].size(); j++){
            grid[i][j].x = grid[i][j].x * 20 + 10;
            grid[i][j].y = grid[i][j].y * 20 + 10;
        }
    }
}

// Start DWA.
void Widget::DWAStart(){
    //Simulate robot.
    robotInitialized = true;
    CoordinateTransformation();

    // Set initial attribute.
    // DWA::setAttribute(3.0, -0.5, 40.0 * M_PI / 180.0, 0.2, 40.0 * M_PI / 180.0, 0.01, 0.1 * M_PI / 180.0, 0.1, 3.0, 0.15, 1.0, 1.0, 0.001, 14.0);

    if(!result.empty()){
        for(Node* node : result){
            qDebug() << "(" << node -> x << ", " << node -> y << ")";
        }
        qDebug() << "length: " << result.size() - 1;
    }

    robot.x = result[0] -> x;
    robot.y = result[0] -> y;
    result.pop_front();



    // Set timer.
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Widget::updateRobot);
    // Fixedupdate time.
    timer -> start(50);
}

// Update robot by timer.
void Widget::updateRobot(){
    if(result.empty()){
        return;
    }
    auto [v, omega] = DWA::calculateControl(robot, obstacles, {result[0] -> x, result[0] -> y});
    // Update robot state.
    // v = 0.5;
    //  omega = 0.5;
    robot.theta += omega * DWA::getDt();
    robot.x += v * cos(robot.theta) * DWA::getDt();
    robot.y += v * sin(robot.theta) * DWA::getDt();
    robot.v = v;
    robot.omega = omega;
    if(resultInRange()){
        result.pop_front();
    }
    update();
}

// Draw robot.
void Widget::drawRobot(QPainter& painter){
    painter.setBrush(Qt::yellow);
    painter.drawEllipse(robot.x - 5, robot.y - 5, 10, 10);
    painter.drawLine(robot.x, robot.y, robot.x + 40 * cos(robot.theta), robot.y + 40 * sin(robot.theta));
}

// If get result front node.
bool Widget::resultInRange() const{
    Node* front = result[0];
    return pow((robot.x - front -> x), 2) + pow((robot.y - front -> y), 2) <= 25;
}

// Shift widgets.
void Widget::showSEOWidget(){
    pStack -> setCurrentIndex(0);
    mode = 0;
}

void Widget::showRobotWidget(){
    pStack -> setCurrentIndex(1);
    mode = 1;
}

void Widget::showMoveOWidget(){
    pStack -> setCurrentIndex(2);
    mode = 2;
}

Widget::~Widget() {}
