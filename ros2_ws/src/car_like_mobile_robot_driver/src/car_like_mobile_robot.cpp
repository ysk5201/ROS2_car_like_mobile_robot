#include "car_like_mobile_robot_driver/car_like_mobile_robot.hpp"


CarLikeMobileRobot::CarLikeMobileRobot() : Node("car_like_mobile_robot"),
    isAtEndPoint(false),
    bezier_data_(),
    Com_(N + 1, std::vector<long long>(N + 1, 0)),
    pre_time_(0.0),
    x_(0.0), y_(0.0), th_(0.0), phi_(0.0),
	s_(0.0), d_(0.0), thetat_(0.0),
	v_(0.0),
    q_search_index_(0.0), is_full_search_(true),
    fl_steering_angle_(0.0), fr_steering_angle_(0.0),
    rl_linear_velocity_(0.0), rr_linear_velocity_(0.0) {
    initializeSubscribers();
    initializePublishers();
    calc_com();
}

CarLikeMobileRobot::~CarLikeMobileRobot() {
    RCLCPP_INFO(this->get_logger(), "CarLikeMobileRobot Node Shutting Down.");
}

void CarLikeMobileRobot::initializeSubscribers() {
    // true_state_variables_sub_ = nh_.subscribe("/true_state_variables", 1, &CarLikeMobileRobot::truePositionCallback, this);
}

void CarLikeMobileRobot::calc_com() {
    const long long MOD = 1000000007;
    Com_[0][0] = 1;
    for (int i = 1; i < N + 1; ++i) {
        Com_[i][0] = 1;
        for (int j = 1; j < N + 1; ++j) {
            Com_[i][j] = (Com_[i-1][j-1] + Com_[i-1][j]) % MOD;
        }
    }
}

void CarLikeMobileRobot::calcDesiredPathParams() {
    calcBezierParameters(); // ベジェ曲線のパラメータを計算 & bezier_data_に格納
}

void CarLikeMobileRobot::getCurrentStateVariables() {
    // コールバック関数
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    phi_ = 0.0;
}

std::array<double, 2> CarLikeMobileRobot::calcControlInput() {

    double x = x_;
    double y = y_;
    double th = th_;
    double phi = phi_;

	double fw1 = 0.2; // 速度の定義[m/s]

    double u1 = 1.0;
    double u2 = 0.0;

    return {u1, u2};
}

void CarLikeMobileRobot::calcCommand(double dt, const std::array<double, 2>& u) {

    double u1 = u[0];
    double u2 = u[1];

    double velocity = u1; // 後輪間中点の速度
    double current_phi = phi_;
    double future_phi = current_phi + (u2 * dt); // 未来の前輪ステアリング角度

    double new_r = WHEEL_BASE / tan(future_phi); // 未来の旋回半径
    double phi_l = atan(WHEEL_BASE / (new_r - (0.5 * TREAD_WIDTH)));
    double phi_r = atan(WHEEL_BASE / (new_r + (0.5 * TREAD_WIDTH)));

    double current_r = WHEEL_BASE / tan(current_phi); // 現在の旋回半径
    double omega = velocity / current_r;              // 後輪間中点の旋回角速度
    double omega_l = (velocity - (0.5 * TREAD_WIDTH * omega)) / WHEEL_RADIUS;
    double omega_r = (velocity + (0.5 * TREAD_WIDTH * omega)) / WHEEL_RADIUS;

    // 未来の前輪ステアリング角度
    fl_steering_angle_ = phi_l;
    fr_steering_angle_ = phi_r;
    // 現在の後輪回転速度
    rl_linear_velocity_ = omega_l;
    rr_linear_velocity_ = omega_r;
}

void CarLikeMobileRobot::publishCommand() {
    publishSteeringAngles(fl_steering_angle_, fr_steering_angle_);
    publishWheelAngularVelocities(rl_linear_velocity_, rr_linear_velocity_);
}

// double CarLikeMobileRobot::Sec(double a) {
//     return (1.0 / cos(a));
// }

// double CarLikeMobileRobot::Arctan(double a, double b) {
//     return (atan2(b,a));
// }

void CarLikeMobileRobot::calcBezierParameters() {
    bezier_data_.clear(); // 以前のデータをクリア
    
    std::vector<double> s_values(Q_DIV_NUM + 1, 0.0);  // 経路長を格納

    for (int j = 0; j <= Q_DIV_NUM; j++) {
        double q = static_cast<double>(j) / Q_DIV_NUM;

        BezierParameters params;
        params.q = q;

        double Rq[5][2] = {};  // ベジェ曲線のqでの微分
        double Rs[5][2] = {};  // sでの微分
        double curv[3] = {};   // 曲率の微分

        calcRqDiff(q, Rq);
        calcRsDiff(Rq, Rs);
        calcCurvature(Rs, curv);

        params.Rx = Rq[0][0];
        params.Ry = Rq[0][1];
        params.dRxdq = Rq[1][0];
        params.dRydq = Rq[1][1];

        params.c = curv[0];
        params.dcds = curv[1];
        params.d2cds2 = curv[2];

        bezier_data_.push_back(params);
    }

    calcArcLength(s_values);  // 経路長sを計算しs_valuesに保存

    //bezier_data_にsを保存
    for (int j = 0; j <= Q_DIV_NUM; j++) {
        bezier_data_[j].s = s_values[j];
        // printf("s=%lf\n", bezier_data_[j].s);
    }

    RCLCPP_INFO(this->get_logger(), "Bezier curve length : %f", bezier_data_[Q_DIV_NUM].s);
}

void CarLikeMobileRobot::calcArcLength(std::vector<double>& s_values) {

	double s_k[S_DIM+1][4], s_q[S_DIM+1][4], s_r[S_DIM+1][4];
	double s_x[3][S_DIM+1];
	double s_x_old[S_DIM+1], s_x_new[S_DIM+1];

	int     i, j, time;
	long	n;
    
	for (int i = 0; i < S_DIM + 1; i ++ ) {
		s_x_old[i] = 0.0;
	}

    double q_max = 1.0;
	double dq = q_max / Q_DIV_NUM; // サンプリング間隔
	
    n = Q_DIV_NUM + 1;

	// 初期値を配列に保存
    s_values[0] = s_x_old[1];  // 初期値

	for (i = 0 ; i < S_DIM+1 ; i++)
		s_q[i][3] = 0.0;
	for (j = 1 ; j < n ; j++) {

		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][0] = dq * (this->*s_f[i])(s_x_old);
			s_r[i][0] = (s_k[i][0] - 2.0 * s_q[i][3]) / 2.0;
			s_x[0][i] = s_x_old[i] + s_r[i][0];
			s_q[i][0] = s_q[i][3] + 3.0 * s_r[i][0] - s_k[i][0] / 2.0;
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][1] = dq * (this->*s_f[i])(s_x[0]);
			s_r[i][1] = (1.0 - sqrt(0.5)) * (s_k[i][1] - s_q[i][0]);
			s_x[1][i] = s_x[0][i] + s_r[i][1];
			s_q[i][1] = s_q[i][0] + 3.0 * s_r[i][1] - (1.0 - sqrt(0.5)) * s_k[i][1];
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][2] = dq * (this->*s_f[i])(s_x[1]);
			s_r[i][2] = (1.0 + sqrt(0.5)) * (s_k[i][2] - s_q[i][1]);
			s_x[2][i] = s_x[1][i] + s_r[i][2];
			s_q[i][2] = s_q[i][1] + 3.0 * s_r[i][2] - (1.0 + sqrt(0.5)) * s_k[i][2];
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][3] = dq * (this->*s_f[i])(s_x[2]);
			s_r[i][3] = (s_k[i][3] - 2.0 * s_q[i][2]) / 6.0;
			s_x_new[i] = s_x[2][i] + s_r[i][3];
			s_q[i][3] = s_q[i][2] + 3.0 * s_r[i][3] - s_k[i][3] / 2.0;
		}

		time = j;
		time = j - time;
		
		if (time == 0 ) {
            s_values[j] = s_x_new[1];  // 計算した経路長を格納
		}

		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_x_old[i] = s_x_new[i];
        }
	}
}

double CarLikeMobileRobot::s_f0(double s_x[S_DIM + 1]) {
	return(1.0);
}

// 経路長の微分方程式(dsdq)
double CarLikeMobileRobot::s_f1(double s_x[S_DIM + 1]) {
	double q = s_x[0];
	double Rq_x = 0.0;
	double Rq_y = 0.0;
	for (int i = 1; i < N + 1; i ++) {
		double weight = nCk(N, i) * i * std::pow(q, i - 1) * std::pow(1 - q, N - i);
		Rq_x += weight * (-B[i - 1][0] + B[i][0]);
		Rq_y += weight * (-B[i - 1][1] + B[i][1]);
    }
	return std::sqrt(std::pow(Rq_x, 2.0) + std::pow(Rq_y, 2.0));
}

long long CarLikeMobileRobot::nCk(int n, int k) {
    if (n < k) {
        RCLCPP_ERROR(this->get_logger(), "Invalid inputs: n (%d) should be greater than or equal to k (%d)", n, k);
        return -1;
    }
    if (n < 0 || k < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid inputs: n (%d) and k (%d) should be non-negative", n, k);
        return -1;
    }
    return Com_[n][k];
}

// ベジェ曲線R(q)のqでの(0~4階)微分プログラム
void CarLikeMobileRobot::calcRqDiff(double q, double Rq[][2]) {
	// nCk計算プログラムの呼び出し
	for (int i = 0; i < N + 1; i ++) {
		for (int j = 0; j < 5; j ++) {
			double weight; // 重みづけの値を定義
			// 0階微分
			if (j == 0) {
				weight = nCk(N, i)*std::pow(q, i)*std::pow(1-q, N-i);
                Rq[j][0] += weight*B[i][0];
                Rq[j][1] += weight*B[i][1];
			}
			// 1階微分
			if (j == 1 && i > 0) {
				weight = nCk(N, i)*i*std::pow(q, i-1)*std::pow(1-q, N-i);
                Rq[j][0] += weight*(-B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(-B[i-1][1]+B[i][1]);
			}
			// 2階微分
			if (j == 2 && i > 1) {
				weight = nCk(N, i)*i*(i-1)*std::pow(q, i-2)*std::pow(1-q, N-i);
                Rq[j][0] += weight*(B[i-2][0]-2*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(B[i-2][1]-2*B[i-1][1]+B[i][1]);
			}
			// 3階微分
			if (j == 3 && i > 2) {
				weight = nCk(N, i)*i*(i-1)*(i-2)*std::pow(q, i-3)*std::pow(1-q, N-i);
                Rq[j][0] += weight*(-B[i-3][0]+3*B[i-2][0]-3*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(-B[i-3][1]+3*B[i-2][1]-3*B[i-1][1]+B[i][1]);
			}
			// 4階微分
			if (j == 4 && i > 3) {
				weight = nCk(N, i)*i*(i-1)*(i-2)*(i-3)*std::pow(q, i-4)*std::pow(1-q, N-i);
                Rq[j][0] += weight*(B[i-4][0]-4*B[i-3][0]+6*B[i-2][0]-4*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(B[i-4][1]-4*B[i-3][1]+6*B[i-2][1]-4*B[i-1][1]+B[i][1]);
			}
		}
	}
}

// ベジェ曲線R(q)のsでの(0~4階)微分プログラム
void CarLikeMobileRobot::calcRsDiff(double Rq[][2], double Rs[][2]) {

	// R(q)のqでの1階微分した時のノルム(dsdq)
	double dsdq = sqrt(std::pow(Rq[1][0], 2) + std::pow(Rq[1][1], 2));

	// R(q)をsで(0~4階)微分
 // Rs[0][0] = Rq[0][0];
 // Rs[0][1] = Rq[0][1];
    Rs[1][0] = Rq[1][0]/dsdq;
    Rs[1][1] = Rq[1][1]/dsdq;
    Rs[2][0] = Rq[1][1]*(Rq[1][1]*Rq[2][0]-Rq[1][0]*Rq[2][1])/std::pow(dsdq,4);
    Rs[2][1] = Rq[1][0]*(-Rq[1][1]*Rq[2][0]+Rq[1][0]*Rq[2][1])/std::pow(dsdq,4);
    Rs[3][0] = (std::pow(Rq[1][1],3)*(-3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])+Rq[1][0]*Rq[1][0]*Rq[1][1]*(5*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])-Rq[1][0]*Rq[1][1]*Rq[1][1]*(4*Rq[2][0]*Rq[2][0]-3*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])-std::pow(Rq[1][0],3)*(Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1]))/std::pow(dsdq,7);
    Rs[3][1] = (-std::pow(Rq[1][1],3)*Rq[2][0]*Rq[2][0]+Rq[1][0]*Rq[1][1]*Rq[1][1]*(5*Rq[2][0]*Rq[2][1]-Rq[1][1]*Rq[3][0])-std::pow(Rq[1][0],3)*(3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])+std::pow(Rq[1][0],4)*Rq[3][1]+Rq[1][0]*Rq[1][0]*Rq[1][1]*(3*Rq[2][0]*Rq[2][0]-4*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1]))/std::pow(dsdq,7);
    Rs[4][0] = (std::pow(Rq[1][1],4)*(-4*std::pow(Rq[2][0],3)+Rq[2][0]*(15*Rq[2][1]*Rq[2][1]-4*Rq[1][1]*Rq[3][1])+Rq[1][1]*(-6*Rq[2][1]*Rq[3][0]+Rq[1][1]*Rq[4][0]))+std::pow(Rq[1][0],4)*(9*Rq[2][0]*(Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])+Rq[1][1]*(7*Rq[2][1]*Rq[3][0]+Rq[1][1]*Rq[4][0]))+Rq[1][0]*Rq[1][0]*Rq[1][1]*Rq[1][1]*(24*std::pow(Rq[2][0],3)+Rq[2][0]*(-60*Rq[2][1]*Rq[2][1]+5*Rq[1][1]*Rq[3][1])+Rq[1][1]*(Rq[2][1]*Rq[3][0]+2*Rq[1][1]*Rq[4][0]))-std::pow(Rq[1][0],5)*(3*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[4][1])+std::pow(Rq[1][0],3)*Rq[1][1]*(-33*Rq[2][0]*Rq[2][0]*Rq[2][1]+13*std::pow(Rq[2][1],3)-13*Rq[1][1]*Rq[2][0]*Rq[3][0]+7*Rq[1][1]*Rq[2][1]*Rq[3][1]-2*Rq[1][1]*Rq[1][1]*Rq[4][1])-Rq[1][0]*std::pow(Rq[1][1],3)*(-51*Rq[2][0]*Rq[2][0]*Rq[2][1]+15*std::pow(Rq[2][1],3)+13*Rq[1][1]*Rq[2][0]*Rq[3][0]-10*Rq[1][1]*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[1][1]*Rq[4][1]))/std::pow(dsdq,10);
    Rs[4][1] = (-3*std::pow(Rq[1][1],4)*Rq[2][0]*(-3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])-std::pow(Rq[1][0],5)*(4*Rq[2][1]*Rq[3][0]+6*Rq[2][0]*Rq[3][1]+Rq[1][1]*Rq[4][0])+std::pow(Rq[1][0],3)*Rq[1][1]*(-15*std::pow(Rq[2][0],3)+Rq[2][0]*(51*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])+Rq[1][1]*(5*Rq[2][1]*Rq[3][0]-2*Rq[1][1]*Rq[4][0]))+Rq[1][0]*std::pow(Rq[1][1],3)*(13*std::pow(Rq[2][0],3)+Rq[2][0]*(-33*Rq[2][1]*Rq[2][1]+7*Rq[1][1]*Rq[3][1])+Rq[1][1]*(9*Rq[2][1]*Rq[3][0]-Rq[1][1]*Rq[4][0]))+std::pow(Rq[1][0],6)*Rq[4][1]+Rq[1][0]*Rq[1][0]*Rq[1][1]*Rq[1][1]*(-60*Rq[2][0]*Rq[2][0]*Rq[2][1]+24*std::pow(Rq[2][1],3)+7*Rq[1][1]*Rq[2][0]*Rq[3][0]-13*Rq[1][1]*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[1][1]*Rq[4][1])+std::pow(Rq[1][0],4)*(15*Rq[2][0]*Rq[2][0]*Rq[2][1]-4*std::pow(Rq[2][1],3)+10*Rq[1][1]*Rq[2][0]*Rq[3][0]-13*Rq[1][1]*Rq[2][1]*Rq[3][1]+2*Rq[1][1]*Rq[1][1]*Rq[4][1]))/std::pow(dsdq,10);
}

// 曲率cのsでの(0~2階)微分プログラム
void CarLikeMobileRobot::calcCurvature(double Rs[][2], double curv[3]) {
    curv[0] = Rs[2][0] * (-Rs[1][1])
            + Rs[2][1] * Rs[1][0];
    curv[1] = (Rs[3][0] + Rs[1][0] * std::pow(curv[0], 2)) * (-Rs[1][1])
            + (Rs[3][1] + Rs[1][1] * std::pow(curv[0], 2)) * Rs[1][0];
    curv[2] = (Rs[4][0] + (-Rs[1][1]) * std::pow(curv[0], 3) + 3 * Rs[1][0] * curv[0] * curv[1]) * (-Rs[1][1])
            + (Rs[4][1] + Rs[1][0] * std::pow(curv[0], 3) + 3 * Rs[1][1] * curv[0] * curv[1]) * Rs[1][0];
}

// void CarLikeMobileRobot::Runge(double t) {

//     double k[RUNGE_DIM + 1][4], r[RUNGE_DIM + 1][4];
//     static double q[RUNGE_DIM + 1][4];
//     double x[3][RUNGE_DIM + 1];
//     double x_old[RUNGE_DIM + 1], x_new[RUNGE_DIM + 1];

// 	int i;
// 	double dt = t - pre_time_;

//     x_old[0]  = t;
//     x_old[1]  = x_;
//     x_old[2]  = y_;
//     x_old[3]  = th_;
//     x_old[4]  = phi_;

//     if (t == 0) {
//         for (int i = 0; i < RUNGE_DIM + 1; ++i) {
//             q[i][3] = 0.0;
//         }
//     }

//     for (i = 0 ; i < RUNGE_DIM + 1 ; i++) {
//         k[i][0] = dt * (this->*f[i])(x_old);
//         r[i][0] = (k[i][0] - 2.0 * q[i][3]) / 2.0;
//         x[0][i] = x_old[i] + r[i][0];
//         q[i][0] = q[i][3] + 3.0 * r[i][0] - k[i][0] / 2.0;
//     }
//     for (i = 0 ; i < RUNGE_DIM + 1 ; i++) {
//         k[i][1] = dt * (this->*f[i])(x[0]);
//         r[i][1] = (1.0 - sqrt(0.5)) * (k[i][1] - q[i][0]);
//         x[1][i] = x[0][i] + r[i][1];
//         q[i][1] = q[i][0] + 3.0 * r[i][1] - (1.0 - sqrt(0.5)) * k[i][1];
//     }
//     for (i = 0 ; i < RUNGE_DIM + 1 ; i++) {
//         k[i][2] = dt * (this->*f[i])(x[1]);
//         r[i][2] = (1.0 + sqrt(0.5)) * (k[i][2] - q[i][1]);
//         x[2][i] = x[1][i] + r[i][2];
//         q[i][2] = q[i][1] + 3.0 * r[i][2] - (1.0 + sqrt(0.5)) * k[i][2];
//     }
//     for (i = 0 ; i < RUNGE_DIM + 1 ; i++) {
//         k[i][3] = dt * (this->*f[i])(x[2]);
//         r[i][3] = (k[i][3] - 2.0 * q[i][2]) / 6.0;
//         x_new[i] = x[2][i] + r[i][3];
//         q[i][3] = q[i][2] + 3.0 * r[i][3] - k[i][3] / 2.0;
//     }

//     // 計算結果をメンバ変数に代入
// 	x_   = x_new[1];
//     y_   = x_new[2];
//     th_  = x_new[3];
//     phi_ = x_new[4];
// }

// double CarLikeMobileRobot::f0(double x[RUNGE_DIM + 1]) {
// 	return(1.0);
// }

// double CarLikeMobileRobot::f1(double x[RUNGE_DIM + 1]) {
// 	double u1 = u_[0];
// 	double ret = u1 * cos(th_);
// 	return(ret);
// }

// double CarLikeMobileRobot::f2(double x[RUNGE_DIM + 1]) {
//     double u1 = u_[0];
// 	double ret = u1 * sin(th_);
// 	return(ret);
// }

// double CarLikeMobileRobot::f3(double x[RUNGE_DIM + 1]) {
// 	double u1 = u_[0];
// 	double ret = u1 * tan(th_) / WHEEL_BASE;
// 	return(ret);
// }

// double CarLikeMobileRobot::f4(double x[RUNGE_DIM + 1]) {
//     double u2 = u_[1];
// 	double ret = u2;
// 	return(ret);
// }

// 後輪間中点の位置(x,y)を受け取ってベジェ曲線のs, d, thetat, c, dcds, d2cds2をそれぞれポインタで返す関数
// void CarLikeMobileRobot::calcBezierParam(double x, double y, double& s, double& d, double& thetat, double& c, double& dcds, double& d2cds2) {

// 	// Ps探索後の最適なqを保存
//     double q = findPs(x, y);

// 	if (q == 1) {
// 		isAtEndPoint = true;
// 	}

//     // ROS_INFO("bezier_q %lf", q);

// 	double Rq[5][2]={0}; // Rをqで(0~4回)微分した配列
//     double Rs[5][2]={0}; // Rをsで(0~4回)微分した配列
//     double curv[3]={0};  // 曲率cをsで(0~2回)微分した配列

// 	calcRqDiff(q, Rq);      // ベジェ曲線R(q)のqでの(0~4階)微分プログラム
// 	calcRsDiff(Rq, Rs);  // ベジェ曲線R(q)のsでの(0~4階)微分プログラム
// 	calcCurvature(Rs, curv); // 曲率cのsでの(0~2階)微分プログラム

// 	double dsdq = sqrt(std::pow(Rq[1][0], 2.0) + std::pow(Rq[1][1], 2.0));

// 	// 単位ベクトルeの(x, y)成分
//     double dRxds = Rq[1][0]/dsdq;
//     double dRyds = Rq[1][1]/dsdq;

// 	// 単位ベクトルn(e+90deg)の(x, y)成分
// 	double nx = -dRyds;
// 	double ny = dRxds;

// 	// qに対応する経路長配列s_の行数を導出
// 	int value = round(q * Q_DIV_NUM);

//     s       = s_array_[value];                           // 経路長
// 	d       = (x-Rq[0][0])*nx + (y-Rq[0][1])*ny; // 経路上の点Psと、後輪間中点へのベクトルとe、との内積をdとして返す
//     thetat  = atan2(dRyds, dRxds);                 // 経路上の点Psにおける接線の方向角
//     c       = curv[0];                             // 曲率c(s)
//     dcds    = curv[1];                             // 曲率c(s)の一階微分
//     d2cds2  = curv[2];                             // 曲率c(s)の二階微分
// }

// double CarLikeMobileRobot::findPs(double x, double y) {

//     // 後輪間中点とベジェ曲線との距離の最小値を無限大で初期化
//     double min = min = std::numeric_limits<double>::max();
//     double save_q = 0.0; // 最適なベジェ曲線のパラメータq

//     // 部分探索の基準点
//     int start, stop;

//     if (is_full_search_) {
//         start = 0;
//         stop = Q_DIV_NUM;
//         is_full_search_ = false;  // 次回以降は部分探索
//     } else {
//         // 部分探索の範囲を設定
//         start = std::max(0, q_search_index_ - PARTIAL);
//         stop = std::min(Q_DIV_NUM, q_search_index_ + PARTIAL);
//     }

// 	// Ps探索
//     for (int j = start; j <= stop; j++) {
//         double q = static_cast<double>(j) / Q_DIV_NUM;

//         // ベジェ曲線上の座標 R[0] (Rx), R[1] (Ry) を計算
//         double R[2] = {0.0, 0.0};
//         for (int i = 0; i <= N; i++) {
//             double weight = nCk(N, i) * std::pow(q, i) * std::pow(1 - q, N - i);
//             R[0] += weight * B[i][0];
//             R[1] += weight * B[i][1];
//         }

//         // 2乗距離を計算
//         double d = std::pow(x - R[0], 2.0) + std::pow(y - R[1], 2.0);
//         if (d < min) {
//             min = d;
//             save_q = q;
//             q_search_index_ = j;
//         }
//     }

// 	return save_q;
// }


// std::vector<double> CarLikeMobileRobot::getOutputVariables(double t) {
// 	std::array<double, 2> form_closure_score = evaluateFormClosureScore(true_vehicle1_pos_, true_vehicle2_pos_, true_vehicle3_pos_);
//     return {t, x_, y_, th_, phi_, s_, d_, dd, th1_ - thetat_, thetap1d};
// }



void CarLikeMobileRobot::initializePublishers() {
    front_steering_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/front_steering_position_controller/commands", 1);
    rear_wheel_pub_      = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rear_wheel_speed_controller/commands", 1);
}

void CarLikeMobileRobot::publishSteeringAngles(double phi_l, double phi_r) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {phi_l, phi_r};
    front_steering_pub_->publish(msg);
}

void CarLikeMobileRobot::publishWheelAngularVelocities(double omega_l, double omega_r) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {omega_l, omega_r};
    rear_wheel_pub_->publish(msg);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto car_like_mobile_robot = std::make_shared<CarLikeMobileRobot>();
    
    car_like_mobile_robot->calcDesiredPathParams();

    rclcpp::Rate loop_rate(50);

    char input;
    std::cout << "Press 's' to start publishing\n";
    std::cin >> input;

    if (input != 's') {
        std::cout << "Exiting program.\n";
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::Time start_time = car_like_mobile_robot->now();
    double pre_t = 0.0;

    while (rclcpp::ok() && input == 's') {

        double t = (car_like_mobile_robot->now() - start_time).seconds();
        double dt = t - pre_t;

        car_like_mobile_robot->getCurrentStateVariables();                   // 状態変数を取得
        std::array<double, 2> u = car_like_mobile_robot->calcControlInput(); // 状態変数から制御入力を導出
        car_like_mobile_robot->calcCommand(dt, u);                           // 制御入力を後輪回転角速度とステアリング角度に変換
        car_like_mobile_robot->publishCommand();                             // 後輪回転角速度とステアリング角度をpublsih

        if (car_like_mobile_robot->isAtEndPoint) {
            break;
        }

        pre_t = t;

        rclcpp::spin_some(car_like_mobile_robot);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}