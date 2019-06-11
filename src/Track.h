//
//  Track.h
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/26.
//

#pragma once

#ifndef Track_h
#define Track_h

#include "HeaderCV.h"
#include "ParamManager.hpp"

using namespace cv;

class Track {
public:
    std::vector<Point2f> point;
    std::vector<Point2f> disp;
    std::vector<float> hog;
    std::vector<float> hof;
    std::vector<float> mbhX;
    std::vector<float> mbhY;
    int index;
    
    Track(const Point2f &point_, const ParamManager::TrackInfo &trackInfo,
          const ParamManager::DescInfo &hogInfo, const ParamManager::DescInfo &hofInfo,
          const ParamManager::DescInfo &mbhInfo)
            : point(trackInfo.length + 1), disp(trackInfo.length),
            hog(hogInfo.dim * trackInfo.length),
            hof(hofInfo.dim * trackInfo.length),
            mbhX(mbhInfo.dim * trackInfo.length),
            mbhY(mbhInfo.dim * trackInfo.length) {
        index = 0;
        point[0] = point_;
    }
    
    void addPoint(const Point2f &point_) {
        index++;
        point[index] = point_;
    }
};



#endif /* Track_h */
