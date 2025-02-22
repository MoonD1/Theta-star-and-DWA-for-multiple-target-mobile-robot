#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <math.h>
#include <queue>
#include <unordered_set>
#include <QPushButton>


// The nodes in grid.
struct Node{
    int x, y;
    bool isObstacle;
    int g, h, f;
    Node* parent;

    Node(): x(0), y(0), isObstacle(false), g(INT_MAX), h(0), f(INT_MAX), parent(nullptr){}
    Node(int x, int y):x(x), y(y), isObstacle(false), g(INT_MAX), h(0), f(0), parent(nullptr){}


};

// Overload operator ">" for A*'s priority_queue.
struct CompareNode{
    bool operator()(const Node* a, const Node* b) const{
        return a -> f > b -> f;
    }
};

// bool operator > (const Node* self, const Node* other){
//     return self -> f > other -> f;
// }
// bool operator < (const Node& other) const{
//     return f < other.f;
// }

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget* parent = nullptr);

    // The function of receiving paint events passed in event.
    void paintEvent(QPaintEvent* event) override;
    // The function of mouse click event.
    void mousePressEvent(QMouseEvent* event) override;

    ~Widget();

private:
    QVector<QVector<Node>> grid;
    Node* startNode = nullptr;
    QVector<Node*> endNode;
    /*
     *  nodes : startNode + endNodes
     *  pathTable[i][j] : the path of nodes from i to i + j + 1
     *  For example:
     *      pathTable[0][0] means the path from nodes[0] to nodes[1] (from startNode to endNode[0])
     *      pathTable[0][1] means the path from nodes[0] to nodes[2] (from startNode to endNode[1])
     *      pathTable[2][3] means the path from nodes[2] to nodes[6] (from endNode[1] to endNode[5])
    */
    QVector<Node*> nodes;
    QVector<QVector<QVector<Node*>>> pathTable;
    int bestPathSize = INT_MAX;
    QVector<int> bestPath;
    QVector<Node*> result;

    // Initialize.
    void initializeGrid(int width, int height);
    // Make initialize grid.
    void drawGrid(QPainter& painter);
    // Use Manhattan distance as the heuristic function.
    int heuristic(const Node* node, const Node* endNode);
    // Reset g/h/f/parent
    void reset();
    // Get A* path.
    QVector<Node*> findPath(Node* node1, Node* node2);
    // Calculate path size.
    int calculatePath(int start, int end);
    // Get all possible path using backtracking algorithm and choose the shortest path.
    void getFullArrangement(QVector<int>& nums, QVector<bool>& used, QVector<int>& path);
    // If click searchPathButton.
    void searchPathButtonClicked();

};
#endif // WIDGET_H
