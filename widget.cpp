#include "widget.h"
#define WIDTH 600
#define HEIGHT 400

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    initializeGrid(20, 20);

    // Set window title.
    this->setWindowTitle("Path Simulation");

    // Set window size.
    // this->resize(WIDTH, HEIGHT);
    // this->setMinimumSize(WIDTH, HEIGHT);
    this->setFixedSize(WIDTH, HEIGHT);

    // Add push button to search path
    QPushButton* searchPathButton = new QPushButton("Search Path", this);
    searchPathButton -> setGeometry(450, 180, 100, 40);
    connect(searchPathButton, &QPushButton::clicked, this, &Widget::searchPathButtonClicked);
}

// Initialize.
void Widget::initializeGrid(int width, int height){
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
    drawGrid(painter);
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
    if(event -> pos().x() > 400 || event -> pos().y() > 400){
        return;
    }
    int x = event -> pos().x() / 20;
    int y = event -> pos().y() / 20;
    // Use leftbutton set obstacle.
    if(event -> button() == Qt::LeftButton){
        grid[x][y].isObstacle = !grid[x][y].isObstacle;
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
    update();
}

// Use Manhattan distance as the heuristic function.
int Widget::heuristic(const Node* node, const Node* endNode){
    return std::abs(node -> x - endNode -> x) + std::abs(node -> y - endNode -> y);
}

// Get A* path.
QVector<Node*> Widget::findPath(Node* node1, Node* node2){
    if(!node1 || !node2){
        return {};
    }
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openQueue;
    std::unordered_set<Node*> openSet;
    std::unordered_set<Node*> closeSet;
    // Initialize.
    node1 -> g = 0;
    node1 -> h = node1 -> f = heuristic(node1, node2);
    openQueue.push(node1);
    openSet.insert(node1);
    // A*.
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
                if(openSet.find(nextNode) == openSet.end()){
                    openQueue.push(nextNode);
                    openSet.insert(nextNode);
                }
            }
        }
    }

    return {};
}

// If click searchPathButton
void Widget::searchPathButtonClicked(){
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
}

Widget::~Widget() {}
