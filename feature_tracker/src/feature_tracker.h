#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

/*
 *  inBorder函数： 看一看输入进来的特征点位置是否超过图像便捷
 *  reduceVector函数： 两处调用，一次时在做完LK光流后进行vector的剔除，一次是在通过基本矩阵剔除outlier后进行vector的剔除
 */
bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();                                       // hs: 构造函数

    void readImage(const cv::Mat &_img);                    // hs: 单目相机执行特征点获取，剔除等主函数

    void setMask();                                         // hs: 设置掩码，例如鱼眼相机是取中间圆

    void addPoints();                                       // hs: 增加特征点，设置其跟踪次数以及id初始化（-1）

    bool updateID(unsigned int i);                          // hs: 更新新加入特征点的id

    void readIntrinsicParameter(const string &calib_file);  // hs: 读取相机内参，待解决

    void rejectWithF();                                     // hs: 利用基本矩阵排除部分outlier

    void showUndistortion(const string &name);              // hs: 待解决

    vector<cv::Point2f> undistortedPoints();                // hs: 待解决

    cv::Mat mask;                                           // hs: 掩码
    cv::Mat fisheye_mask;                                   // hs: 鱼眼相机掩码
    cv::Mat prev_img, cur_img, forw_img;                    // hs: forw表示当前帧，cur表示光流上一帧，prev表示上次发布的帧
    vector<cv::Point2f> n_pts;                              // hs: 为了保持光流跟踪的特征点的数量，需不断补充特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;        // hs: forw表示当前帧提取的特征点，cur表示光流上一帧提取的特征点，prev表示上次发布的特征点
    vector<int> ids;                                        // hs: 表示特征点的id
    vector<int> track_cnt;                                  // hs: 对每个特征点的跟踪次数进行统计,好像没怎么使用啊？？？
    camodocal::CameraPtr m_camera;                          // hs: 相机模型（未解决）

    static int n_id;                                        // hs: 特征点id更新变量
};
