#include "dwa.h"
#include <QDebug>

DWA::DWA() {}

// Set initial attribute.

double DWA::max_speed = 3.0; // m/s
double DWA::min_speed = -0.5; // m/s
double DWA::max_yaw_rate = 40.0 * M_PI / 180.0; // rad/s
double DWA::max_accel = 0.2; // m/ss
double DWA::max_delta_yaw_rate = 40.0 * M_PI / 180.0; // rad/ss
double DWA::v_resolution = 0.01; // m/s
double DWA::yaw_rate_resolution = 0.1 * M_PI / 180.0; // rad/s
double DWA::dt = 0.5; // s
double DWA::predict_time = 3; // s
double DWA::to_goal_cost_coefficient = 0.15;
double DWA::speed_cost_coefficient = 1.0;
double DWA::obstacle_cost_coefficient = 1.0;
double DWA::robot_stuck_flag_cons = 0.001;
double DWA::robot_radius = 14.0;

void DWA::setAttribute(double max_v, double min_v, double max_y_r, double max_a, double max_d_y_r, double v_res, double y_r_res, double t, double pre_t, double t_g_c_coe,
                       double s_c_coe, double o_c_coe, double r_s_f_cons, double r_rad){
    DWA::max_speed = max_v; // m/s
    DWA::min_speed = min_v; // m/s
    DWA::max_yaw_rate = max_y_r; // rad/s
    DWA::max_accel = max_a; // m/ss
    DWA::max_delta_yaw_rate = max_d_y_r; // rad/ss
    DWA::v_resolution = v_res; // m/s
    DWA::yaw_rate_resolution = y_r_res; // rad/s
    DWA::dt = t; // s
    DWA::predict_time = pre_t; // s
    DWA::to_goal_cost_coefficient = t_g_c_coe;
    DWA::speed_cost_coefficient = s_c_coe;
    DWA::obstacle_cost_coefficient = o_c_coe;
    DWA::robot_stuck_flag_cons = r_s_f_cons;
    DWA::robot_radius = r_rad;
}

// Get dt
double DWA::getDt(){
    return DWA::dt;
}


// Algorithm.
std::pair<double, double> DWA::calculateControl(const RobotState& robot, const QSet<std::pair<double, double>>& obstacles, const std::pair<double, double> goal){
    // qDebug() << goal.first << " " << goal.second;
    QVector<std::pair<double, double>> dynamicWindow = calcDynamicWindow(robot);
    // Initialization.
    std::pair<double, double> bestControl = {0.0, 0.0};
    QVector<RobotState> bestTrajectory;
    double minCost = std::numeric_limits<double>::infinity();
    // Traverse the dynamic window.
    for(double v = dynamicWindow[0].first; v <= dynamicWindow[1].first; v += v_resolution){
        for(double omega = dynamicWindow[0].second; omega <= dynamicWindow[1].second; omega += yaw_rate_resolution){
            QVector<RobotState> trajectory = calcTrajectory(robot, v, omega);
            // Calculate cost.
            double toGoalCost = to_goal_cost_coefficient * calcToGoalCost(trajectory, goal);
            double speedCost = speed_cost_coefficient * (max_speed - trajectory.back().v);
            double obstacleCost = obstacle_cost_coefficient * calcObstacleCost(trajectory, obstacles);

            // qDebug() << toGoalCost << " " << speedCost << " " << obstacleCost;
            // qDebug() << obstacleCost;
            double finalCost = toGoalCost + obstacleCost + speedCost;


            if(minCost > finalCost){
                // qDebug() << finalCost;
                minCost = finalCost;
                bestControl = {v, omega};
                bestTrajectory = trajectory;
                // To ensure the robot do not get stuck in.
                if(abs(bestControl.first) < robot_stuck_flag_cons && abs(robot.v) < robot_stuck_flag_cons){
                    bestControl.second = -max_delta_yaw_rate;
                }
            }
        }
    }

    return bestControl;
}

// Calculate dynamic window.
QVector<std::pair<double, double>> DWA::calcDynamicWindow(const RobotState& robot){
    // The range of dynamic window.
    double minV = min_speed;
    double maxV = max_speed;
    double minOmega = -max_yaw_rate;
    double maxOmega = max_yaw_rate;
    // The range of cur robot dynamic window.
    double minVWithAcceleration = robot.v - max_accel * dt;
    double maxVWithAcceleration = robot.v + max_accel * dt;
    double minOmegaWithAcceleration = robot.omega - max_delta_yaw_rate * dt;
    double maxOmegaWithAcceleration = robot.omega + max_delta_yaw_rate * dt;
    // Get dynamic window range.
    std::pair<double, double> getMin = {std::max(minV, minVWithAcceleration), std::max(minOmega, minOmegaWithAcceleration)};
    std::pair<double, double> getMax = {std::min(maxV, maxVWithAcceleration), std::min(maxOmega, maxOmegaWithAcceleration)};
    return {getMin, getMax};
}

// Calculate the trajectory by v and omega.
QVector<RobotState> DWA::calcTrajectory(const RobotState& robot, const double& v, const double& omega){
    QVector<RobotState> trajectory;
    RobotState tempRobot = robot;
    double time = 0;
    while(time < predict_time){
        tempRobot.theta += omega * dt;
        tempRobot.x += v * cos(tempRobot.theta) * dt;
        tempRobot.y += v * sin(tempRobot.theta) * dt;
        tempRobot.v = v;
        tempRobot.omega = omega;

        trajectory.push_back(tempRobot);
        time += dt;
    }
    return trajectory;
}

// Calculate the cost to goal.
double DWA::calcToGoalCost(const QVector<RobotState>& trajectory, const std::pair<double, double>& goal){
    double dx = goal.first - trajectory.back().x;
    double dy = goal.second - trajectory.back().y;
    double errorAngle = atan2(dy, dx);
    double costAngle = errorAngle - trajectory.back().theta;
    double cost = abs(atan2(sin(costAngle), cos(costAngle)));

    return cost;
}

// Calculate the cost of obstacle.
double DWA::calcObstacleCost(const QVector<RobotState>& trajectory, const QSet<std::pair<double, double>>& obstacles){
    double min_r = std::numeric_limits<double>::infinity();
    for(const auto& point : trajectory){
        for(const auto& obstacle : obstacles){
            double dx = point.x - obstacle.first;
            double dy = point.y - obstacle.second;
            double distance = sqrt(dx * dx + dy * dy);
            min_r = std::min(min_r, distance);
            if(distance < robot_radius ){
                return std::numeric_limits<double>::infinity();
            }
        }
    }
    return 1.0 / min_r;
}

DWA::~DWA(){}
