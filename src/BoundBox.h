//
//  BoundBox.h
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#pragma once

#ifndef BoundBox_h
#define BoundBox_h

#include "HeaderCV.h"
#include "ParamManager.hpp"

using namespace cv;

class BoundBox {
public:
    Point2f TopLeft;
    Point2f BottomRight;
    float confidence;
    
    BoundBox(float a1, float a2, float a3, float a4, float a5) {
        TopLeft.x = a1;
        TopLeft.y = a2;
        BottomRight.x = a3;
        BottomRight.y = a4;
        confidence = a5;
    }
};


#endif /* BoundBox_h */
