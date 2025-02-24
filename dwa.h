#ifndef DWA_H
#define DWA_H

#include <QPointF>
#include <QVector>
#include <QSet>


// Simulate robot
struct RobotState{
    // position
    double x;
    double y;
    // direction
    double theta;
    // velocity
    double v;
    // angular velocity
    double omega;

    RobotState(): x(0.0), y(0.0), theta(0.0), v(0.0), omega(0.0){}
    RobotState(double x, double y): x(x), y(y), theta(0.0), v(0.0), omega(0.0){}
};

class DWA
{
public:
    DWA();

    // Algorithm.
    static std::pair<double, double> calculateControl(const RobotState& robot, const QSet<std::pair<double, double>>& obstacles, const std::pair<double, double> goal);

private:

    // Calculate dynamic window.
    static QVector<std::pair<double, double>> calcDynamicWindow(const RobotState& robot);
    // Calculate the trajectory by v and omega.
    static QVector<std::pair<double, double>> calcTrajectory(const RobotState& robot, const double& v, const double& omega);
    // Calculate the cost to goal.
    static double calcToGoalCost(const QVector<std::pair<double, double>> trajectory, const std::pair<double, double> goal);
    // Calculate the cost of obstacle.
    static double calcObstacleCost(const QVector<std::pair<double, double>> trajectory, const QSet<std::pair<double, double>>& obstacles);

    ~DWA();
};

#endif // DWA_H
