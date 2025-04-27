#include "setrobot.h"
#include "dwa.h"


SetRobot::SetRobot(QWidget *parent)
    : QWidget{parent}
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    QHBoxLayout* line1 = new QHBoxLayout();
    QHBoxLayout* line2 = new QHBoxLayout();
    QHBoxLayout* line3 = new QHBoxLayout();
    QHBoxLayout* line4 = new QHBoxLayout();
    QHBoxLayout* line5 = new QHBoxLayout();
    QHBoxLayout* line6 = new QHBoxLayout();
    QHBoxLayout* line7 = new QHBoxLayout();
    QHBoxLayout* line8 = new QHBoxLayout();
    QHBoxLayout* line9 = new QHBoxLayout();
    QHBoxLayout* line10 = new QHBoxLayout();
    QHBoxLayout* line11 = new QHBoxLayout();
    QHBoxLayout* line12 = new QHBoxLayout();
    QHBoxLayout* line13 = new QHBoxLayout();
    QHBoxLayout* line14 = new QHBoxLayout();
    QHBoxLayout* line15 = new QHBoxLayout();
    QHBoxLayout* button = new QHBoxLayout();

    QLabel* pi1 = new QLabel("* PI / 180.0", this);
    QLabel* pi2 = new QLabel("* PI / 180.0", this);
    QLabel* pi3 = new QLabel("* PI / 180.0", this);

    QLabel* maxSpeedLabel = new QLabel("max_speed:", this);
    QLabel* minSpeedLabel = new QLabel("min_speed:", this);
    QLabel* maxYawRateLabel = new QLabel("max_yaw_rate:", this);
    QLabel* maxAccelLabel = new QLabel("max_accel:", this);
    QLabel* maxDeltaYawRateLabel = new QLabel("max_delta_yaw_rate:", this);
    QLabel* vResolutionLabel = new QLabel("v_resolution:", this);
    QLabel* yawRateResolutionLabel = new QLabel("yaw_rate_resolution:", this);
    QLabel* dtLabel = new QLabel("dt:", this);
    QLabel* predictTimeLabel = new QLabel("predict_time:", this);
    QLabel* toGoalCostCoefficientLabel = new QLabel("to_goal_cost_coe:", this);
    QLabel* speedCostCoefficientLabel = new QLabel("speed_cost_coe:", this);
    QLabel* obstacleCostCoefficientLabel = new QLabel("obstacle_cost_coe:", this);
    QLabel* mobileObsCostCoefficientLabel = new QLabel("mobile_obs_cost_coe:", this);
    QLabel* robotStuckFlagConsLabel = new QLabel("robot_stuck_flag_cons:", this);
    QLabel* robotRadiusLabel = new QLabel("robot_radius:", this);

    maxSpeedEdit = new QLineEdit(this);
    minSpeedEdit = new QLineEdit(this);
    maxYawRateEdit = new QLineEdit(this);
    maxAccelEdit = new QLineEdit(this);
    maxDeltaYawRateEdit = new QLineEdit(this);
    vResolutionEdit = new QLineEdit(this);
    yawRateResolutionEdit = new QLineEdit(this);
    dtEdit = new QLineEdit(this);
    predictTimeEdit = new QLineEdit(this);
    toGoalCostCoefficientEdit = new QLineEdit(this);
    speedCostCoefficientEdit = new QLineEdit(this);
    obstacleCostCoefficientEdit = new QLineEdit(this);
    mobileObsCostCoefficientEdit = new QLineEdit(this);
    robotStuckFlagConsEdit = new QLineEdit(this);
    robotRadiusEdit = new QLineEdit(this);

    maxSpeedEdit -> setPlaceholderText("m/s, default 3.0");
    minSpeedEdit -> setPlaceholderText("m/s, default -0.5");
    maxYawRateEdit -> setPlaceholderText("rad/s, default 40.0");
    maxAccelEdit -> setPlaceholderText("m/ss, default 0.2");
    maxDeltaYawRateEdit -> setPlaceholderText("rad/ss, default 40.0");
    vResolutionEdit -> setPlaceholderText("m/s, default 0.01");
    yawRateResolutionEdit -> setPlaceholderText("rad/s, default 0.1");
    dtEdit -> setPlaceholderText("s, default 0.5");
    predictTimeEdit -> setPlaceholderText("s, default 3.0");
    toGoalCostCoefficientEdit -> setPlaceholderText("default 0.15");
    speedCostCoefficientEdit -> setPlaceholderText("default 1.0");
    obstacleCostCoefficientEdit -> setPlaceholderText("default 1.0");
    mobileObsCostCoefficientEdit -> setPlaceholderText("default 100.0");
    robotStuckFlagConsEdit -> setPlaceholderText("default 0.001");
    robotRadiusEdit -> setPlaceholderText("m, default 15.0");

    QPushButton* apply = new QPushButton("Apply");
    apply -> setFixedSize(200, 25);
    connect(apply, &QPushButton::clicked, this, &SetRobot::setRobot);

    line1 -> addWidget(maxSpeedLabel);
    line1 -> addWidget(maxSpeedEdit);
    line2 -> addWidget(minSpeedLabel);
    line2 -> addWidget(minSpeedEdit);
    line3 -> addWidget(maxYawRateLabel);
    line3 -> addWidget(maxYawRateEdit);
    line3 -> addWidget(pi1);
    line4 -> addWidget(maxAccelLabel);
    line4 -> addWidget(maxAccelEdit);
    line5 -> addWidget(maxDeltaYawRateLabel);
    line5 -> addWidget(maxDeltaYawRateEdit);
    line5 -> addWidget(pi2);
    line6 -> addWidget(vResolutionLabel);
    line6 -> addWidget(vResolutionEdit);
    line7 -> addWidget(yawRateResolutionLabel);
    line7 -> addWidget(yawRateResolutionEdit);
    line7 -> addWidget(pi3);
    line8 -> addWidget(dtLabel);
    line8 -> addWidget(dtEdit);
    line9 -> addWidget(predictTimeLabel);
    line9 -> addWidget(predictTimeEdit);
    line10 -> addWidget(toGoalCostCoefficientLabel);
    line10 -> addWidget(toGoalCostCoefficientEdit);
    line11 -> addWidget(speedCostCoefficientLabel);
    line11 -> addWidget(speedCostCoefficientEdit);
    line12 -> addWidget(obstacleCostCoefficientLabel);
    line12 -> addWidget(obstacleCostCoefficientEdit);
    line13 -> addWidget(mobileObsCostCoefficientLabel);
    line13 -> addWidget(mobileObsCostCoefficientEdit);
    line14 -> addWidget(robotStuckFlagConsLabel);
    line14 -> addWidget(robotStuckFlagConsEdit);
    line15 -> addWidget(robotRadiusLabel);
    line15 -> addWidget(robotRadiusEdit);
    button -> addWidget(apply);
    button -> setAlignment(apply, Qt::AlignRight);

    layout -> addLayout(line1);
    layout -> addLayout(line2);
    layout -> addLayout(line3);
    layout -> addLayout(line4);
    layout -> addLayout(line5);
    layout -> addLayout(line6);
    layout -> addLayout(line7);
    layout -> addLayout(line8);
    layout -> addLayout(line9);
    layout -> addLayout(line10);
    layout -> addLayout(line11);
    layout -> addLayout(line12);
    layout -> addLayout(line13);
    layout -> addLayout(line14);
    layout -> addLayout(line15);
    layout -> addLayout(button);

}


void SetRobot::setRobot(){
    bool isEmpty[15];
    double max_speed = maxSpeedEdit -> text().toDouble(&isEmpty[0]);
    double min_speed = minSpeedEdit -> text().toDouble(&isEmpty[1]);
    double max_yaw_rate = maxYawRateEdit -> text().toDouble(&isEmpty[2]);
    double max_accel = maxAccelEdit -> text().toDouble(&isEmpty[3]);
    double max_delta_yaw_rate = maxDeltaYawRateEdit -> text().toDouble(&isEmpty[4]);
    double v_resolution = vResolutionEdit -> text().toDouble(&isEmpty[5]);
    double yaw_rate_resolution = yawRateResolutionEdit -> text().toDouble(&isEmpty[6]);
    double dt = dtEdit -> text().toDouble(&isEmpty[7]);
    double predict_time = predictTimeEdit -> text().toDouble(&isEmpty[8]);
    double to_goal_cost_coefficient = toGoalCostCoefficientEdit -> text().toDouble(&isEmpty[9]);
    double speed_cost_coefficient = speedCostCoefficientEdit -> text().toDouble(&isEmpty[10]);
    double obstacle_cost_coefficient = obstacleCostCoefficientEdit -> text().toDouble(&isEmpty[11]);
    double mobile_obs_cost_coefficient = mobileObsCostCoefficientEdit -> text().toDouble(&isEmpty[12]);
    double robot_stuck_flag_cons = robotStuckFlagConsEdit -> text().toDouble(&isEmpty[13]);
    double robot_radius = robotRadiusEdit -> text().toDouble(&isEmpty[14]);

    DWA::setAttribute(isEmpty[0] ? max_speed : 3.0,
                      isEmpty[1] ? min_speed : -0.5,
                      isEmpty[2] ? max_yaw_rate * M_PI / 180.0 : 40.0 * M_PI / 180.0,
                      isEmpty[3] ? max_accel : 0.2,
                      isEmpty[4] ? max_delta_yaw_rate * M_PI / 180.0 : 40.0 * M_PI / 180.0,
                      isEmpty[5] ? v_resolution : 0.01,
                      isEmpty[6] ? yaw_rate_resolution * M_PI / 180.0 : 0.1 * M_PI / 180.0,
                      isEmpty[7] ? dt : 0.5,
                      isEmpty[8] ? predict_time : 3,
                      isEmpty[9] ? to_goal_cost_coefficient : 0.15,
                      isEmpty[10] ? speed_cost_coefficient : 1.0,
                      isEmpty[11] ? obstacle_cost_coefficient : 1.0,
                      isEmpty[12] ? mobile_obs_cost_coefficient : 100.0,
                      isEmpty[13] ? robot_stuck_flag_cons : 0.001,
                      isEmpty[14] ? robot_radius : 15.0
                      );

}

SetRobot::~SetRobot(){}
