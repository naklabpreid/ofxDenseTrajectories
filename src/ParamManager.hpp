//
//  ParamManager.hpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#ifndef ParamManager_hpp
#define ParamManager_hpp

#include <stdio.h>
#include <vector>
//#include "cereal/archives/json.hpp"
//#include "cereal/cereal.hpp"

class ParamManager {
private:
  static ParamManager *instance;
  ParamManager(std::string name);
  ~ParamManager() {}
  ParamManager(const ParamManager &) {}
  ParamManager &operator=(const ParamManager &);

public:
  static ParamManager *initialize();
  static ParamManager *getInstance();
  static void destroy();

public:
  bool loadParam(std::string name);
  bool saveParam(std::string name);
  void setDefaultValue();

  // video parameter
  int start_frame = 0;
  int end_frame = INT_MAX;
  int scale_num = 8;
  float scale_stride = 1.4142;
  char *bb_file = NULL;

  // parameters for descriptors
  int patch_size = 32;
  int nxy_cell = 2;
  int nt_cell = 3;
  float epsilon = 0.05;
  float min_flow = 0.4;

  // parameters for tracking
  double quality = 0.001;
  int min_distance = 5;
  int init_gap = 1;
  int track_length = 15;

  // parameters for rejecting trajectory
  float min_var = 1.73205;
  float max_var = 50;
  float max_dis = 20;

  typedef struct {
    int x; // top left corner
    int y;
    int width;
    int height;
  } RectInfo;

  typedef struct {
    int width; // resolution of the video
    int height;
    int length; // number of frames
  } SeqInfo;

  typedef struct {
    int length; // length of the trajectory
    int gap;    // initialization gap for feature re-sampling
  } TrackInfo;

  typedef struct {
    int nBins; // number of bins for vector quantization
    bool isHof;
    int nxCells; // number of cells in x direction
    int nyCells;
    int ntCells;
    int dim;    // dimension of the descriptor
    int height; // size of the block for computing the descriptor
    int width;
  } DescInfo;

  typedef struct {
    int height;
    int width;
    int nBins;
    float *desc;
  } DescMat;

private:
};

#endif /* ParamManager_hpp */
