//
//  Initialize.cpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#include "Initialize.hpp"
#include "ofVideoPlayer.h"

using namespace cv;

void Initialize::InitTrackInfo(ParamManager::TrackInfo *trackInfo,
                               int track_length, int init_gap) {
  trackInfo->length = track_length;
  trackInfo->gap = init_gap;
}

//-------------------------
void Initialize::InitDescInfo(ParamManager::DescInfo *descInfo, int nBins,
                              bool isHof, int size, int nxy_cell, int nt_cell) {
  descInfo->nBins = nBins;
  descInfo->isHof = isHof;
  descInfo->nxCells = nxy_cell;
  descInfo->nyCells = nxy_cell;
  descInfo->ntCells = nt_cell;
  descInfo->dim = nBins * nxy_cell * nxy_cell;
  descInfo->height = size;
  descInfo->width = size;
}

//-------------------------
ParamManager::DescMat *Initialize::InitDescMat(int height, int width,
                                               int nBins) {
  ParamManager::DescMat *descMat =
      (ParamManager::DescMat *)malloc(sizeof(ParamManager::DescMat));
  descMat->height = height;
  descMat->width = width;
  descMat->nBins = nBins;

  long size = height * width * nBins;
  descMat->desc = (float *)malloc(size * sizeof(float));
  memset(descMat->desc, 0, size * sizeof(float));
  return descMat;
}

void Initialize::ReleDescMat(ParamManager::DescMat *descMat) {
  free(descMat->desc);
  free(descMat);
}

void Initialize::InitSeqInfo(ParamManager::SeqInfo *seqInfo,
                             std::vector<ofImage> imgSeq) {
    
    seqInfo->length = imgSeq.size();
    seqInfo->width = imgSeq.at(0).getWidth();
    seqInfo->height = imgSeq.at(0).getHeight();
    std::cout << "Seq info : length = " << seqInfo->length << ", width = " << seqInfo->width << ", height = " << seqInfo->height << std::endl;
}
