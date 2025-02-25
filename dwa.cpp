#include "dwa.h"
#include "qdebug.h"

DWA::DWA() {}

// Algorithm.
std::pair<double, double> DWA::calculateControl(const RobotState& robot, const QSet<std::pair<double, double>>& obstacles, const std::pair<double, double> goal){
    // qDebug() << goal.first << " " << goal.second;
    QVector<std::pair<double, double>> dynamicWindow = calcDynamicWindow(robot);
    // Initialization.
    std::pair<double, double> bestControl = {0.0, 0.0};
    QVector<std::pair<double, double>> bestTrajectory;
    double bestScore = std::numeric_limits<double>::lowest();
    // Traverse the dynamic window.
    for(double v = dynamicWindow[0].first; v <= dynamicWindow[1].first; v += 0.1){
        for(double omega = dynamicWindow[0].second; omega <= dynamicWindow[1].second; omega += 0.01){
            QVector<std::pair<double, double>> trajectory = calcTrajectory(robot, v, omega);
            // Calculate cost.
            double toGoalCost = calcToGoalCost(trajectory, goal);
            double obstacleCost = calcObstacleCost(trajectory, obstacles);
            double speedCost = 1.0 - v;

            double score = 0.15 * toGoalCost - 3 * obstacleCost - speedCost;

            if(score > bestScore){
                // qDebug() << score;
                bestScore = score;
                bestControl = {v, omega};
                bestTrajectory = trajectory;
            }
        }
    }

    return bestControl;
}

// Calculate dynamic window.
QVector<std::pair<double, double>> DWA::calcDynamicWindow(const RobotState& robot){
    // The range of dynamic window.
    double minV = 0.0;
    double maxV = 1.0;
    double minOmega = -1.0;
    double maxOmega = 1.0;
    // The range of cur robot dynamic window.
    double minVWithAcceleration = robot.v - 0.1;
    double maxVWithAcceleration = robot.v + 0.1;
    // double minOmegaWithAcceleration = robot.omega - 1;
    // double maxOmegaWithAcceleration = robot.omega + 1;
    // Get dynamic window range.
    std::pair<double, double> getMin = {std::max(minV, minVWithAcceleration), minOmega};
    std::pair<double, double> getMax = {std::min(maxV, maxVWithAcceleration), maxOmega};
    return {getMin, getMax};
}

// Calculate the trajectory by v and omega.
QVector<std::pair<double, double>> DWA::calcTrajectory(const RobotState& robot, const double& v, const double& omega){
    QVector<std::pair<double, double>> trajectory;
    RobotState tempRobot = robot;
    // simulate 50 steps.
    for(int i = 0; i < 50; i++){
        tempRobot.x += v * cos(tempRobot.theta);
        tempRobot.y += v * sin(tempRobot.theta);
        tempRobot.theta += omega;
        trajectory.push_back({tempRobot.x, tempRobot.y});
    }
    return trajectory;
}

// Calculate the cost to goal.
double DWA::calcToGoalCost(const QVector<std::pair<double, double>> trajectory, const std::pair<double, double> goal){
    double dx = trajectory.back().first - goal.first;
    double dy = trajectory.back().second - goal.second;
    return -sqrt(dx * dx + dy * dy);
}

// Calculate the cost of obstacle.
double DWA::calcObstacleCost(const QVector<std::pair<double, double>> trajectory, const QSet<std::pair<double, double>>& obstacles){
    double cost = 0.0;
    for(const auto& point : trajectory){
        for(const auto& obstacle : obstacles){
            double dx = point.first - obstacle.first;
            double dy = point.second - obstacle.second;
            double distance = sqrt(dx * dx + dy * dy);
            if(distance < 19.2){
                cost += 1.0 / distance;
            }
        }
    }
    return cost;
}

DWA::~DWA(){}
