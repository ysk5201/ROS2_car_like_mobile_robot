#pragma once

#ifndef CAR_LIKE_MOBILE_ROBOT_HPP_
#define CAR_LIKE_MOBILE_ROBOT_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


class CarLikeMobileRobot : public rclcpp::Node {
public:
    // Constractor and Destructor
    CarLikeMobileRobot();
    ~CarLikeMobileRobot();

    // Member functions
    void calcDesiredPathParams();
    // void getCurrentStateVariables();
    void calcControlInput();
    void Runge(rclcpp::Time t);
    void calcCommand(); // 関数名要検討
    void publishCommand();

    // Member variables
    bool isAtEndPoint;

private:
    // constants
    static constexpr int S_DIM = 1;                // 経路長の数値積分の次元数
    static constexpr int Q_DIV_NUM = 100000;       // ベジェ曲線q[0,1]の分割数
    static constexpr int STATE_VARIABLE_DIM = 4;   // 車両型移動ロボットの状態変数
    static constexpr int RUNGE_DIM = 1;            // Runge()で計算する状態変数の数
    static constexpr int PARTIAL = 500;             // 部分探索の範囲[q_search_index_ - PARTIAL, q_search_index_ + PARTIAL]
    static constexpr double PI = 3.141592653589793;
    static constexpr double WHEEL_BASE = 1.0;      // 車軸間距離(m) (URDFと統一)
    static constexpr double TREAD_WIDTH = 0.856;   // 左右車輪間距離(m) (URDFと統一)
    static constexpr double WHEEL_RADIUS = 0.3;    // 車輪半径(m) (URDFと統一)

    // 制御点を定義
    std::vector<std::array<double,2>> B;    //制御点
    int N; // ベジェ曲線の次元数
    
    // フィードバックゲイン
    static constexpr double P11	= -3.0;
    static constexpr double P12	= -4.0;
    static constexpr double P13	= -2.0;

    // Member Variables
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_variables_sub_;
    
    // ベジェ曲線の構造体を定義
    struct BezierParameters {
        double q;            // ベジェ曲線パラメータ
        double s;            // 経路長
        double thetat;       // 点Psにおける接線方位角
        double c;            // 曲率
        double dcds;         // 曲率の一階微分
        double d2cds2;       // 曲率の二階微分
        double Rx, Ry;       // ベジェ曲線上の座標(x, y)
        double dRxdq, dRydq; // (x, y)の一階微分
    };
    
    std::vector<BezierParameters> bezier_data_; // ベジェ曲線の計算結果を保持するメンバ変数
    
    rclcpp::Time pre_time_;
    double x_, y_, th_, phi_;
    double future_phi_;

    std::array<double, 2> u_; // 制御入力u1, u2

    int q_search_index_;  // 前回の探索で見つけた最適なqのインデックス
    bool is_full_search_; // Ps探索(全探索, 部分探索)

    double fl_steering_angle_, fr_steering_angle_;
    double rl_linear_velocity_, rr_linear_velocity_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
    
    
    // Member functions
    void initParams();
    void initializeSubscribers();
    void stateVariablesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void calcBezierParameters(); // ベジェ曲線パラメータを計算する関数
	void calcArcLength(std::vector<double>& s_values); // ベジェ曲線の経路長を計算
    double s_f0(double s_x[S_DIM + 1]);
    double s_f1(double s_x[S_DIM + 1]);
    typedef double (CarLikeMobileRobot::*s_FUNC)(double*);
    s_FUNC s_f[S_DIM+1] = {&CarLikeMobileRobot::s_f0, &CarLikeMobileRobot::s_f1};
    long long nCk(int n, int k); // 二項係数の計算をする関数
    void calcRqDiff(double q, double Rq[][2]);
    void calcRsDiff(double Rq[][2], double Rs[][2]);
    void calcCurvature(double Rs[][2], double curv[3]);

    void findPs(double x, double y);

    double f0(double x[RUNGE_DIM + 1]);  // t
    double f1(double x[RUNGE_DIM + 1]);  // phi
    typedef double (CarLikeMobileRobot::*FUNC)(double*);
    FUNC f[RUNGE_DIM + 1] = {&CarLikeMobileRobot::f0, &CarLikeMobileRobot::f1};
    
    void initializePublishers();
    void publishSteeringAngles(double phi_l, double phi_r);
    void publishWheelAngularVelocities(double omega_l, double omega_r);

};

#endif // CAR_LIKE_MOBILE_ROBOT_HPP_