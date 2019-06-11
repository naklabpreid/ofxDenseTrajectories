//
//  Initialize.hpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#ifndef Initialize_hpp
#define Initialize_hpp

#include "HeaderCV.h"
#include "ParamManager.hpp"
#include <stdio.h>

class Initialize {
public:
  Initialize(){};
  ~Initialize(){};

  void InitTrackInfo(ParamManager::TrackInfo *trackInfo, int track_length,
                     int init_gap);

  void InitDescInfo(ParamManager::DescInfo *descInfo, int nBins, bool isHof,
                    int size, int nxy_cell, int nt_cell);

  ParamManager::DescMat *InitDescMat(int height, int width, int nBins);

  void ReleDescMat(ParamManager::DescMat *descMat);

  void InitSeqInfo(ParamManager::SeqInfo *seqInfo, std::vector<ofImage> imgSeq);
};
#endif /* Initialize_hpp */
