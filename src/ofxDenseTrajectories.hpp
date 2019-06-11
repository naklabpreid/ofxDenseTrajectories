//
//  ofxDenseTrajectories.hpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/25.
//

#ifndef ofxDenseTrajectories_hpp
#define ofxDenseTrajectories_hpp

#include "BoundBox.h"
#include "Descriptors.hpp"
#include "Frame.h"
#include "HeaderCV.h"
#include "Initialize.hpp"
#include "OpticalFlow.hpp"
#include "ParamManager.hpp"
#include "Track.h"
#include "ofVideoPlayer.h"

class ofxDenseTrajectories {
public:
    ofxDenseTrajectories();
    ~ofxDenseTrajectories(){};
    
    typedef enum { HOG, HOF, MBHx, MBHy } DescType;
    
    void setup(std::string video_path);
    void update();
    void drawTrajectories();
    void writeCSV(std::string csvPath, DescType type);

    bool isFinished();
    bool isInitialized = false;
    cv::Mat getFrame(){ return frame; }
    
private:
    struct trackDraw{
        ofPoint pos;
        int index;
        int scale;
    };
    
    std::vector<trackDraw> trackDrawVec;
    
    ParamManager *param;
    Initialize init;
    Descriptors desc;
    OpticalFlow optFlow;
    
    void procFirstFrame();
    void proc();
    
    ParamManager::TrackInfo trackInfo;
    ParamManager::DescInfo hogInfo, hofInfo, mbhInfo;
    ParamManager::SeqInfo seqInfo;
    
    int init_counter = 0;
    int frame_num = 0;
    
    cv::VideoCapture capture;
    ofVideoPlayer vidPlayer;
    std::vector<ofImage> imageVec;
    
    Mat frame;
    Mat image, prev_grey, grey;
    std::vector<Mat> prev_grey_pyr, grey_pyr, flow_pyr, flow_warp_pyr;
    std::vector<Mat> prev_poly_pyr, poly_pyr, poly_warp_pyr;
    
    std::vector<Point2f> prev_pts_flow, pts_flow;
    std::vector<Point2f> prev_pts_surf, pts_surf;
    std::vector<Point2f> prev_pts_all, pts_all;
    
    std::vector<KeyPoint> prev_kpts_surf, kpts_surf;
    Mat prev_desc_surf, desc_surf;
    Mat flow, human_mask;
    
    SurfFeatureDetector detector_surf;
    SurfDescriptorExtractor extractor_surf;
    
    std::vector<float> fscales;
    std::vector<cv::Size> sizes;
    
    std::vector<std::list<Track>> xyScaleTracks;
    std::vector<Frame> bb_list;
    
    std::vector<std::vector<float>> Store_HOG;
    std::vector<std::vector<float>> Store_HOF;
    std::vector<std::vector<float>> Store_MBHx;
    std::vector<std::vector<float>> Store_MBHy;
    void clearDesc(DescType type);
};

#endif /* ofxDenseTrajectories_hpp */

