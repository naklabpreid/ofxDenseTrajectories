//
//  Frame.h
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#pragma once

#ifndef Frame_h
#define Frame_h

#include "BoundBox.h"
#include "HeaderCV.h"
#include "ParamManager.hpp"

using namespace cv;

class Frame {
public:
    int frameID;
    std::vector<BoundBox> BBs;
    
    Frame(const int &frame_) {
        frameID = frame_;
        BBs.clear();
    }
};

#endif /* Frame_h */
