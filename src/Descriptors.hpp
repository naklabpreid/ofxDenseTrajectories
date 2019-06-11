//
//  Descriptors.hpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#ifndef Descriptors_hpp
#define Descriptors_hpp

#include "BoundBox.h"
#include "Frame.h"
#include "HeaderCV.h"
#include "ParamManager.hpp"
#include "Track.h"
#include <stdio.h>

using namespace cv;
class Descriptors {
public:
    Descriptors();
    ~Descriptors() {}
    
    ParamManager *param;
    
    void GetRect(const Point2f &point, ParamManager::RectInfo &rect,
                 const int width, const int height,
                 const ParamManager::DescInfo &descInfo);
    
    void BuildDescMat(const Mat &xComp, const Mat &yComp, float *desc,
                      const ParamManager::DescInfo &descInfo);
    
    void GetDesc(const ParamManager::DescMat *descMat,
                 ParamManager::RectInfo &rect, ParamManager::DescInfo descInfo,
                 std::vector<float> &desc, const int index);
    
    void HogComp(const Mat &img, float *desc, ParamManager::DescInfo &descInfo);
    void HofComp(const Mat &flow, float *desc, ParamManager::DescInfo &descInfo);
    void MbhComp(const Mat &flow, float *descX, float *descY,
                 ParamManager::DescInfo &descInfo);
    bool IsValid(std::vector<Point2f> &track, float &mean_x, float &mean_y,
                 float &var_x, float &var_y, float &length);
    
    bool IsCameraMotion(std::vector<Point2f> &disp);
    
    void DenseSample(const Mat &grey, std::vector<Point2f> &points,
                     const double quality, const int min_distance);
    
    void InitPry(const Mat &frame, std::vector<float> &scales,
                 std::vector<cv::Size> &sizes);
    void BuildPry(const std::vector<cv::Size> &sizes, const int type,
                  std::vector<Mat> &grey_pyr);
    void DrawTrack(const std::vector<Point2f> &point, const int index,
                   const float scale, Mat &image);
    void PrintDesc(std::vector<float> &desc, ParamManager::DescInfo &descInfo,
                   ParamManager::TrackInfo &trackInfo);
    void StoreDesc(std::vector<float>& desc, ParamManager::DescInfo& descInfo, ParamManager::TrackInfo& trackInfo, std::vector<std::vector<float> > &storeVec);
    
    void LoadBoundBox(char *file, std::vector<Frame> &bb_list);
    void InitMaskWithBox(Mat &mask, std::vector<BoundBox> &bbs);
    void MyWarpPerspective(Mat &prev_src, Mat &src, Mat &dst, Mat &M0, int flags,
                           int borderType, const cv::Scalar &borderValue);
    
    void ComputeMatch(const std::vector<KeyPoint> &prev_kpts,
                      const std::vector<KeyPoint> &kpts, const Mat &prev_desc,
                      const Mat &desc, std::vector<Point2f> &prev_pts,
                      std::vector<Point2f> &pts);
    
    void MergeMatch(const std::vector<Point2f> &prev_pts1,
                    const std::vector<Point2f> &pts1,
                    const std::vector<Point2f> &prev_pts2,
                    const std::vector<Point2f> &pts2,
                    std::vector<Point2f> &prev_pts_all,
                    std::vector<Point2f> &pts_all);
    
    void MatchFromFlow(const Mat &prev_grey, const Mat &flow,
                       std::vector<Point2f> &prev_pts, std::vector<Point2f> &pts,
                       const Mat &mask);
};

#endif /* Descriptors_hpp */

