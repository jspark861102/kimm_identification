//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "kimm_phri_msgs/ObjectParameter.h"
#include "dynamic_reconfigure/server.h"

// Tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Object estimation
#include <../../kimm_object_estimation/include/kimm_object_estimation/main/extendedkalman.hpp>
#include <../../kimm_object_estimation/include/kimm_object_estimation/objdyn/object_dynamics.hpp>
#include <kimm_identification/ekf_paramConfig.h>

ros::Publisher object_parameter_pub_;

double time_, dt_;
tf::TransformBroadcaster* br_;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
void accelerationCallback(const geometry_msgs::Twist::ConstPtr& msg);
void FextCallback(const geometry_msgs::Wrench::ConstPtr& msg);
void glocalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void paramCallback(const kimm_phri_msgs::ObjectParameter::ConstPtr& msg);

//////////////////// object estimation /////////////////
// EKF ekf(0.001,A, H, Q, R, P, h);
EKF * ekf;
Objdyn objdyn;

double n_param, m_FT;
Eigen::MatrixXd A, H, Q, R, P;
Eigen::VectorXd h, FT_measured, param, robot_g_local_;
Vector6d vel_param, acc_param;
kimm_phri_msgs::ObjectParameter param_true;

void getObjParam_init();
void getObjParam();
void ObjectParameter_pub();
double saturation(double x, double limit);
bool is_ekf_init_, estimation_start_;

// Dynamic reconfigure    
std::unique_ptr<dynamic_reconfigure::Server<kimm_identification::ekf_paramConfig>> ekf_param_;
void ekfParamCallback(kimm_identification::ekf_paramConfig& config, uint32_t level);
////////////////////////////////////////////////////////

void keyboard_event();
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};

