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

    Node(): x(0), y(0), isObstacle(false), g(INT_MAX), h(0), f(0), parent(nullptr){}
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
    // Initialize.
    void initializeGrid(int width, int height);
    // Make initialize grid.
    void drawGrid(QPainter& painter);
    // Use Manhattan distance as the heuristic function.
    int heuristic(const Node* node, const Node* endNode);
    // Get A* path.
    QVector<Node*> findPath(Node* node1, Node* node2);
    // If click searchPathButton
    void searchPathButtonClicked();

};
#endif // WIDGET_H
