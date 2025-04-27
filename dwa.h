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

    // Set attribute.
    static void setAttribute(double max_v, double min_v, double max_y_r, double max_a, double max_d_y_r, double v_res, double y_r_res, double t, double pre_t, double t_g_c_coe,
                             double s_c_coe, double o_c_coe, double m_o_c_coe, double r_s_f_cons, double r_rad);

    // Get dt
    static double getDt();

private:

    // Calculate dynamic window.
    static QVector<std::pair<double, double>> calcDynamicWindow(const RobotState& robot);
    // Calculate the trajectory by v and omega.
    static QVector<RobotState> calcTrajectory(const RobotState& robot, const double& v, const double& omega);
    // Calculate the cost to goal.
    static double calcToGoalCost(const QVector<RobotState>& trajectory, const std::pair<double, double>& goal);
    // Calculate the cost of obstacle.
    static double calcObstacleCost(const QVector<RobotState>& trajectory, const QSet<std::pair<double, double>>& obstacles);

    static double calcMobileObsCost(const QVector<RobotState>& trajectory);

    ~DWA();



    static double max_speed;
    static double min_speed;
    static double max_yaw_rate;
    static double max_accel;
    static double max_delta_yaw_rate;

    static double v_resolution;
    static double yaw_rate_resolution;

    static double dt;
    static double predict_time;

    static double to_goal_cost_coefficient;
    static double speed_cost_coefficient;
    static double obstacle_cost_coefficient;
    static double mobile_obs_cost_coefficient;

    static double robot_stuck_flag_cons; // constant to prevent robot stucked
    static double robot_radius;


};

#endif // DWA_H
