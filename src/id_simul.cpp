#include "kimm_identification/id_simul.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{   
    //Ros setting
    ros::init(argc, argv, "kimm_identification");
    ros::NodeHandle n_node;
    
    dt_ = 0.001;
    time_ = 0.0;
    ros::Rate loop_rate(1.0/dt_);
    
    ros::Subscriber velocity = n_node.subscribe("/ns1/real_robot/object_velocity", 5, &velocityCallback, ros::TransportHints().tcpNoDelay(true));                    
    ros::Subscriber acceleratioin = n_node.subscribe("/ns1/real_robot/object_acceleration", 5, &accelerationCallback, ros::TransportHints().tcpNoDelay(true));                    
    ros::Subscriber Fext = n_node.subscribe("/ns1/real_robot/Fext_local_forObjectEstimation", 5, &FextCallback, ros::TransportHints().tcpNoDelay(true));                    
    ros::Subscriber g_locl = n_node.subscribe("/ns1/real_robot/robot_g_local", 5, &glocalCallback, ros::TransportHints().tcpNoDelay(true));                    
    ros::Subscriber param_true = n_node.subscribe("/ns1/real_robot/object_parameter", 5, &paramCallback, ros::TransportHints().tcpNoDelay(true));                    
    
    object_parameter_pub_ = n_node.advertise<kimm_phri_msgs::ObjectParameter>("object_parameter", 5);
    robot_g_local_.resize(3);      
    is_ekf_init_ = false; 
    estimation_start_ = false;

    getObjParam_init();                

    //dynamic_reconfigure    
    ros::NodeHandle ekf_param_node_;    
    ekf_param_node_ = ros::NodeHandle("ekf_param_node");
    ekf_param_ = std::make_unique<dynamic_reconfigure::Server<kimm_identification::ekf_paramConfig>>(ekf_param_node_);
    ekf_param_->setCallback(boost::bind(&ekfParamCallback, _1, _2));    

    while (ros::ok()){                               
        keyboard_event();                  
        
        if (estimation_start_) getObjParam();                   // object estimation
        
        ObjectParameter_pub();                            // data plot for monitoring        

        ros::spinOnce();
        loop_rate.sleep();        

        time_ += dt_;
    }//while

    return 0;
}

void ekfParamCallback(kimm_identification::ekf_paramConfig& config, uint32_t level) {
  Q(0,0) = config.Q0;  
  Q(1,1) = config.Q1;  
  Q(2,2) = config.Q2;  
  Q(3,3) = config.Q3;  
  Q(4,4) = config.Q4;  
  Q(5,5) = config.Q5;  
  Q(6,6) = config.Q6;  
  Q(7,7) = config.Q7;  
  Q(8,8) = config.Q8;  
  Q(9,9) = config.Q9;  

  R(0,0) = config.R0;  
  R(1,1) = config.R1;  
  R(2,2) = config.R2;  
  R(3,3) = config.R3;  
  R(4,4) = config.R4;  
  R(5,5) = config.R5;  
  
  ROS_INFO("ekf parameter from dynamic_reconfigure--------------------------------------------------");
  ROS_INFO_STREAM("Q" << "  " << Q(0,0) << "  " << Q(1,1) << "  " << Q(2,2) << "  " << Q(3,3) );
  ROS_INFO_STREAM("Q" << "  " << Q(4,4) << "  " << Q(5,5) << "  " << Q(6,6) << "  " << Q(7,7) << "  " << Q(8,8) << "  " << Q(9,9));
  ROS_INFO_STREAM("R" << "  " << R(0,0) << "  " << R(1,1) << "  " << R(2,2) << "  " << R(3,3) << "  " << R(4,4) << "  " << R(5,5) );
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){     
    geometry_msgs::Twist msg_tmp;
    msg_tmp = *msg;

    vel_param[0] = msg_tmp.linear.x;
    vel_param[1] = msg_tmp.linear.y;
    vel_param[2] = msg_tmp.linear.z;
    vel_param[3] = msg_tmp.angular.x;
    vel_param[4] = msg_tmp.angular.y;
    vel_param[5] = msg_tmp.angular.z;
}

void accelerationCallback(const geometry_msgs::Twist::ConstPtr& msg){     
    geometry_msgs::Twist msg_tmp;
    msg_tmp = *msg;

    acc_param[0] = msg_tmp.linear.x;
    acc_param[1] = msg_tmp.linear.y;
    acc_param[2] = msg_tmp.linear.z;
    acc_param[3] = msg_tmp.angular.x;
    acc_param[4] = msg_tmp.angular.y;
    acc_param[5] = msg_tmp.angular.z;

    if (acc_param[0] != 0.0) estimation_start_ = true;
}

void FextCallback(const geometry_msgs::Wrench::ConstPtr& msg){     
    geometry_msgs::Wrench msg_tmp;
    msg_tmp = *msg;

    FT_measured[0] = msg_tmp.force.x;
    FT_measured[1] = msg_tmp.force.y;
    FT_measured[2] = msg_tmp.force.z;
    FT_measured[3] = msg_tmp.torque.x;
    FT_measured[4] = msg_tmp.torque.y;
    FT_measured[5] = msg_tmp.torque.z;
}

void glocalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){     
    std_msgs::Float32MultiArray msg_tmp;
    msg_tmp = *msg;

    robot_g_local_(0) = msg_tmp.data[0];
    robot_g_local_(1) = msg_tmp.data[1];
    robot_g_local_(2) = msg_tmp.data[2];
}

void paramCallback(const kimm_phri_msgs::ObjectParameter::ConstPtr& msg){         
    kimm_phri_msgs::ObjectParameter msg_tmp;
    msg_tmp = *msg;

    msg_tmp.com.resize(3);
    msg_tmp.inertia.resize(6);

    param_true.mass = msg_tmp.mass;
    param_true.com[0] = msg_tmp.com[0];
    param_true.com[1] = msg_tmp.com[1];
    param_true.com[2] = msg_tmp.com[2];            
}

void getObjParam(){        
    
    if (!is_ekf_init_) {
        // Initialize the filter  
        ekf->init(time_, param);

        is_ekf_init_ = true;
    }
    
    h = objdyn.h(param, vel_param, acc_param, robot_g_local_); 
    H = objdyn.H(param, vel_param, acc_param, robot_g_local_); 

    ekf->update(FT_measured, dt_, A, H, h, Q, R); //Q & R update from dynamic reconfigure
    param = ekf->state();       
}

void ObjectParameter_pub(){
    kimm_phri_msgs::ObjectParameter objparam_msg;  
    objparam_msg.com.resize(3);
    objparam_msg.inertia.resize(6);      

    objparam_msg.mass = saturation(param[0],5.0);
    
    //local coordinate--------------------//
    objparam_msg.com[0] = saturation(param[1],0.6); 
    objparam_msg.com[1] = saturation(param[2],0.6); 
    objparam_msg.com[2] = saturation(param[3],0.6);  
    //-------------------------------------//

    object_parameter_pub_.publish(objparam_msg);              
}

void getObjParam_init(){
    n_param = 10;
    m_FT = 6;

    A.resize(n_param, n_param); // System dynamics matrix
    H.resize(m_FT,    n_param); // Output matrix
    Q.resize(n_param, n_param); // Process noise covariance
    R.resize(m_FT,    m_FT); // Measurement noise covariance
    P.resize(n_param, n_param); // Estimate error covariance
    h.resize(m_FT,    1); // observation      
    param.resize(n_param);        
    FT_measured.resize(m_FT);        
    param_true.com.resize(3);
    param_true.inertia.resize(6);
    
    A.setIdentity();         //knwon, identity
    Q.setIdentity();         //design parameter
    R.setIdentity();         //design parameter    
    P.setIdentity();         //updated parameter
    h.setZero();             //computed parameter
    H.setZero();             //computed parameter    
    param.setZero();   
    FT_measured.setZero();
    vel_param.setZero();
    acc_param.setZero();       
    
    // Q(0,0) *= 0.01;
    // Q(1,1) *= 0.0001;
    // Q(2,2) *= 0.0001;
    // Q(3,3) *= 0.0001;
    // R *= 100000;
    
    // Construct the filter
    ekf = new EKF(dt_, A, H, Q, R, P, h);        
}  

double saturation(double x, double limit) {
    if (x > limit) return limit;
    else if (x < -limit) return -limit;
    else return x;
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            //------------- basic motion -------------------------------------------------//  
            case 'g': //gravity mode
                msg = 0;                
                cout << " " << endl;
                cout << "Gravity mode" << endl;
                cout << " " << endl;
                break;            
        }
    }
}

