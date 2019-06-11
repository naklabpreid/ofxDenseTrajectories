//
//  ParamManager.cpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#include "ParamManager.hpp"
#include <ofFileUtils.h>
#include <ofLog.h>
#include <ofSystemUtils.h>

ParamManager *ParamManager::instance = nullptr;

ParamManager::ParamManager(std::string name)
    : start_frame(0), end_frame(INT_MAX), scale_num(8) {}

ParamManager *ParamManager::initialize() {
  if (instance != nullptr) {
    ofLogNotice() << "すでに初期化されています。再初期化します。";
    delete instance;
  }
  instance = new ParamManager("default");
  return instance;
}

ParamManager *ParamManager::getInstance() {
  if (instance == nullptr) {
    ofLogError() << "先にinitializeを呼んでください";
    return nullptr;
  }
  return instance;
}

void ParamManager::destroy() {
  if (instance != nullptr) {
    delete instance;
    instance = nullptr;
  }
}

bool ParamManager::loadParam(std::string name) { return true; }

bool ParamManager::saveParam(std::string name) { return true; }

void ParamManager::setDefaultValue() {
  // video parameter
  start_frame = 0;
  end_frame = INT_MAX;
  scale_num = 8;
  scale_stride = 1.4142;
  *bb_file = NULL;

  // parameters for descriptors
  patch_size = 32;
  nxy_cell = 2;
  nt_cell = 3;
  epsilon = 0.05;
  min_flow = 0.4;

  // parameters for tracking
  quality = 0.001;
  min_distance = 5;
  init_gap = 1;
  track_length = 15;

  // parameters for rejecting trajectory
  min_var = 1.73205;
  max_var = 50;
  max_dis = 20;
}
