#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

/* hs: 用于闭环检测*/
struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Vector3d P_old;
    Matrix3d R_old;
    vector<cv::Point2f> measurements;
    vector<int> features_ids; 
    bool relocalized;
    bool relative_pose;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};

class Estimator
{
  public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Vector3d>>> &image, const std_msgs::Header &header);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;				            	// hs: 求解器标志，初始化或者非线性优化
    MarginalizationFlag  marginalization_flag;			// hs: 边缘化标志，边缘化old 或者 second new
    
    Vector3d g;							                // hs： 重力向量
    MatrixXd Ap[2], backup_A;					// hs: no use
    VectorXd bp[2], backup_b;					// hs: no use

    Matrix3d ric[NUM_OF_CAM];					// hs: 相机和IMU之间的旋转
    Vector3d tic[NUM_OF_CAM];					// hs: 相机和IMU之间的平移

    Vector3d Ps[(WINDOW_SIZE + 1)];				// hs: slideWindow 中每一帧的位置
    Vector3d Vs[(WINDOW_SIZE + 1)];				// hs: slideWindow 中每一帧的速度
    Matrix3d Rs[(WINDOW_SIZE + 1)];				// hs: slideWindow 中每一帧的旋转矩阵
    Vector3d Bas[(WINDOW_SIZE + 1)];				// hs: slideWindow 中每一帧的加速度计偏置
    Vector3d Bgs[(WINDOW_SIZE + 1)];				// hs: slideWindow 中每一帧的陀螺偏置

    Matrix3d back_R0, last_R, last_R0;				// hs: slideWindow 中使用
    Vector3d back_P0, last_P, last_P0;				// hs: slideWindow 中使用
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];		// hs: slideWindow 中每一帧的头文件
    
    /* hs :预积分，陀螺和加速度计数据*/
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];	// hs: slideWindow 中每一帧的预积分数据
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];			// hs： slideWindow 中每一帧之间imu帧间隔
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];// hs: slideWindow 中每一帧之间imu加速度数据
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];	// hs: slideWindow 中每一帧之间imu陀螺数据

    int frame_count;
    int sum_of_front, sum_of_back;			// hs: 统计量，边缘化的次数，如果边缘化老的sum_of_front + 1，否则sum_of_back + 1
    int sum_of_outlier, sum_of_invalid;				// hs: no use

    FeatureManager f_manager;					// hs: 特征管理器
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;			// hs: 当需要通过程序进行外参估计时，利用该类进行运算

    bool first_imu;						// hs: 标志，是否为第一个IMU数据
    bool is_valid, is_key;					// hs: no use
    bool failure_occur;						// hs: 检测是否跟踪失败

    vector<Vector3d> point_cloud;				// hs: no use
    vector<Vector3d> margin_cloud;				// hs: no use
    vector<Vector3d> key_poses;					// hs: 用于显示
    double initial_timestamp;					// hs: 初始化时间戳

    /* hs: 用于优化时的参数*/
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    
    /* hs: 用于闭环检测*/
    RetriveData retrive_pose_data, front_pose;
    vector<RetriveData> retrive_data_vector;
    int loop_window_index;
    bool relocalize;
    Vector3d relocalize_t;
    Matrix3d relocalize_r;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

};
