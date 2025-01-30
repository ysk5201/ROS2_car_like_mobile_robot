#pragma once

#ifndef CAR_LIKE_MOBILE_ROBOT_HPP_
#define CAR_LIKE_MOBILE_ROBOT_HPP_

#include <array>
#include <vector>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
// #include "cooperative_transportation_system_pkg/cooperative_transportation_system_constants.hpp"


class CarLikeMobileRobot : public rclcpp::Node {
public:
    // Constractor and Destructor
    CarLikeMobileRobot();
    ~CarLikeMobileRobot();

    // Member functions
    void calcDesiredPathParam();
    void getCurrentStateVariables();
    std::array<double, 2> calcControlInput();
    void calcCommand(double dt, const std::array<double, 2>& u); // 関数名要検討
    void publishCommand();

    // Member variables
    bool isAtEndPoint;

private:
    // constants
    static constexpr int N = 3;                   // 3次ベジェ曲線
    static constexpr int S_DIM = 1;               // 経路長の数値積分の次元数
    static constexpr int Q_DIV_NUM = 100000;      // ベジェ曲線q[0,1]の分割数
    static constexpr int STATE_VARIABLE_DIM = 20; // 協調搬送システムの状態変数
    static constexpr int RUNGE_DIM = 4;           // Runge()で計算する状態変数の数
    static constexpr int PARTIAL = 50; // 部分探索の範囲[last_j_-PARTIAL, last_j_+PARTIAL]
    static constexpr double PI = 3.1415926535897932384626433832795028841971;
    static constexpr double lv = 1.0;              // 車軸間距離[m]

    // 制御点を定義
    const double B[N + 1][2] = {
        { 0.0,  0.0},
        { 8.0,  0.0},
        { 0.0,  8.0},
        { 8.0,  8.0}
    };

    // Member Variables
    // rclcpp::Subscriber true_state_variables_sub_;
    double pre_time_;
    double x_, y_, th_, phi_;
    double s_, d_, thetat_;
    double v_;

    double fl_steering_angle_, fr_steering_angle_;
    double rl_linear_velocity_, rr_linear_velocity_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_pub_;

    // double s_array_[Q_DIV_NUM + 1]; // qに対応する経路長
    // std::vector<std::vector<long long>> Com_; // nCk(Com_[n][k])の値を格納(Com_[N + 1][N + 1])
    
    // bool is_full_search_; // Ps探索(全探索, 部分探索)
    
    // Member functions
    void initializeSubscribers();
    
    // // ベジェ曲線R(q)の微分に使用
    // void calc_com();
    // long long nCk(int n, int k);

	// void calc_S(); // ベジェ曲線の経路長を計算
    // void s_initial(double *q_max, double *dq, double s_x0[S_DIM + 1]);
    // double s_f0(double s_x[S_DIM + 1]);
    // double s_f1(double s_x[S_DIM + 1]);
    // typedef double (CarLikeMobileRobot::*s_FUNC)(double*);
    // s_FUNC s_f[S_DIM+1] = {&CarLikeMobileRobot::s_f0, &CarLikeMobileRobot::s_f1};

    // void truePositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // void setCurrentPosition(double pos_x0, double pos_y0, double pos_th0);
    // void phiCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // double Sec(double a);
    // double Arctan(double a, double b);

    // std::array<double, 2> calc_u(double t, double x, double y, double th, double phi);
    // void calcBezierParam(double x, double y, double& s, double& d, double& thetat, double& c, double& dcds, double& d2cds2);
    // double findPs(double x0, double y0);
    // void calcRqDiff(double q, double Rq[][2]);
    // void calcRsDiff(double q, double Rq[][2], double Rs[][2]);
    // void calcCurvature(double Rs[][2], double curv[3]);

    // void Runge(double t);
    // double f0(double x[RUNGE_DIM + 1]);  // t
    // double f1(double x[RUNGE_DIM + 1]);  // x
    // double f2(double x[RUNGE_DIM + 1]);  // y
    // double f3(double x[RUNGE_DIM + 1]);  // th
    // double f4(double x[RUNGE_DIM + 1]);  // phi
    // typedef double (CarLikeMobileRobot::*FUNC)(double*);
    // FUNC f[RUNGE_DIM + 1] = {&CarLikeMobileRobot::f0,
    //                         &CarLikeMobileRobot::f1,
    //                         &CarLikeMobileRobot::f2,
    //                         &CarLikeMobileRobot::f3,
    //                         &CarLikeMobileRobot::f4,};
    
    void initializePublishers();
    void publishSteeringAngles(double phi_l, double phi_r);
    void publishWheelSpeeds(double omega_l, double omega_r);

};

#endif // CAR_LIKE_MOBILE_ROBOT_HPP_