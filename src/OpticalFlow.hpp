//
//  OpticalFlow.hpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#ifndef OpticalFlow_hpp
#define OpticalFlow_hpp

#include "HeaderCV.h"
#include <stdio.h>

using namespace cv;

class OpticalFlow {
public:
  OpticalFlow(){};
  ~OpticalFlow(){};

  void FarnebackPolyExp(const Mat &src, Mat &dst, int n, double sigma);

  void FarnebackUpdateMatrices(const Mat &_R0, const Mat &_R1, const Mat &_flow,
                               Mat &matM, int _y0, int _y1);

  void FarnebackUpdateFlow_GaussianBlur(const Mat &_R0, const Mat &_R1,
                                        Mat &_flow, Mat &matM, int block_size,
                                        bool update_matrices);

  void MedianBlurFlow(Mat &flow, const int ksize);

  void FarnebackPolyExpPyr(const Mat &img, std::vector<Mat> &poly_exp_pyr,
                           std::vector<float> &fscales, int poly_n,
                           double poly_sigma);

  void calcOpticalFlowFarneback(std::vector<Mat> &prev_poly_exp_pyr,
                                std::vector<Mat> &poly_exp_pyr,
                                std::vector<Mat> &flow_pyr, int winsize,
                                int iterations);
};

#endif /* OpticalFlow_hpp */
