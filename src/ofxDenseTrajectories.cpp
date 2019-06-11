//
//  ofxDenseTrajectories.cpp
//  DenseTrajectories
//
//  Created by 宮下 侑大 on 2019/03/25.
//

#include "ofxDenseTrajectories.hpp"

ofxDenseTrajectories::ofxDenseTrajectories() {
    param->initialize();
    param = ParamManager::getInstance();
}

//-------------------------
void ofxDenseTrajectories::setup(std::string video_path) {
    
    ofLogNotice() << "setup : " << video_path;
    ofDirectory dir;
    dir.listDir(video_path);
    dir.sort();
    imageVec.clear();
    for(int i = 0 ; i < dir.size() ; ++i){
        ofImage img;
        img.load(dir.getPath(i));
        imageVec.push_back(img);
    }
    frame_num = 0;
    
    init.InitTrackInfo(&trackInfo, param->track_length, param->init_gap);
    init.InitDescInfo(&hogInfo, 8, false, param->patch_size, param->nxy_cell,
                      param->nt_cell);
    init.InitDescInfo(&hofInfo, 9, true, param->patch_size, param->nxy_cell,
                      param->nt_cell);
    init.InitDescInfo(&mbhInfo, 8, false, param->patch_size, param->nxy_cell,
                      param->nt_cell);
    
    init.InitSeqInfo(&seqInfo, imageVec);
    if (param->bb_file) {
        desc.LoadBoundBox(param->bb_file, bb_list);
        assert(bb_list.size() == seqInfo.length);
    }
    
    detector_surf.hessianThreshold = 50;
    extractor_surf.extended = true;
    extractor_surf.upright = true;
    
    clearDesc(DescType::HOG);
    clearDesc(DescType::HOF);
    clearDesc(DescType::MBHx);
    clearDesc(DescType::MBHy);

    ofLogNotice() << "setup Finished";
}

//-------------------------
void ofxDenseTrajectories::update() {
    if(frame_num >= seqInfo.length)
        return;
    
    frame = ofxCv::toCv(imageVec.at(frame_num));
    if (frame.empty())
        return;
    
    ofSetWindowTitle("Proc : " + std::to_string(frame_num) + "/" + std::to_string(seqInfo.length));
    if (frame_num == param->start_frame) {
        procFirstFrame();
        ofLogNotice() << "Finished first frame";
    } else {
        proc();
    }

}

bool ofxDenseTrajectories::isFinished() {
    if (frame_num >= seqInfo.length) {
        vidPlayer.stop();
        return true;
    } else {
        return false;
    }
}

//-------------------------
void ofxDenseTrajectories::procFirstFrame() {
    image.create(frame.size(), CV_8UC3);
    grey.create(frame.size(), CV_8UC1);
    prev_grey.create(frame.size(), CV_8UC1);
    
    desc.InitPry(frame, fscales, sizes);
    
    // 前フレームと今フレームの画像ピラミッド生成
    desc.BuildPry(sizes, CV_8UC1, prev_grey_pyr);
    desc.BuildPry(sizes, CV_8UC1, grey_pyr);
    desc.BuildPry(sizes, CV_32FC2, flow_pyr);
    desc.BuildPry(sizes, CV_32FC2, flow_warp_pyr);
    
    // オプティカルフローフィールドの画像ピラミッド生成
    desc.BuildPry(sizes, CV_32FC(5), prev_poly_pyr);
    desc.BuildPry(sizes, CV_32FC(5), poly_pyr);
    desc.BuildPry(sizes, CV_32FC(5), poly_warp_pyr);
    
    // Scale Step vector Resize
    xyScaleTracks.clear();
    xyScaleTracks.resize(param->scale_num);
    
    frame.copyTo(image);
    cvtColor(image, prev_grey, CV_BGR2GRAY);
    
    for (int iScale = 0; iScale < param->scale_num; ++iScale) {
        if (iScale == 0)
            prev_grey.copyTo(prev_grey_pyr[0]);
        else
            resize(prev_grey_pyr[iScale - 1], prev_grey_pyr[iScale],
                   prev_grey_pyr[iScale].size(), 0, 0, INTER_LINEAR);
        
        std::vector<Point2f> points(0);
        desc.DenseSample(prev_grey_pyr[iScale], points, param->quality,
                         param->min_distance);
        
        std::list<Track> &tracks = xyScaleTracks[iScale];
        for (int i = 0; i < points.size(); ++i)
            tracks.push_back(Track(points[i], trackInfo, hogInfo, hofInfo, mbhInfo));
    }
    
    // compute polynomial expansion
    optFlow.FarnebackPolyExpPyr(prev_grey, prev_poly_pyr, fscales, 7, 1.5);
    
    human_mask = Mat::ones(frame.size(), CV_8UC1);
    if (param->bb_file)
        desc.InitMaskWithBox(human_mask, bb_list[frame_num].BBs);
    
    detector_surf.detect(prev_grey, prev_kpts_surf, human_mask);
    extractor_surf.compute(prev_grey, prev_kpts_surf, prev_desc_surf);
    
    frame_num++;
    init_counter = 0;
}

void ofxDenseTrajectories::proc() {
    init_counter++;
    frame.copyTo(image);
    cvtColor(image, grey, CV_BGR2GRAY);
    
    detector_surf.detect(grey, kpts_surf, human_mask);
    extractor_surf.compute(grey, kpts_surf, desc_surf);
    desc.ComputeMatch(prev_kpts_surf, kpts_surf, prev_desc_surf, desc_surf, prev_pts_surf, pts_surf);
    
    // compute optical flow for all scales once
    optFlow.FarnebackPolyExpPyr(grey, poly_pyr, fscales, 7, 1.5);
    optFlow.calcOpticalFlowFarneback(prev_poly_pyr, poly_pyr, flow_pyr, 10, 2);
    
    desc.MatchFromFlow(prev_grey, flow_pyr[0], prev_pts_flow, pts_flow, human_mask);
    desc.MergeMatch(prev_pts_flow, pts_flow, prev_pts_surf, pts_surf, prev_pts_all, pts_all);
    
    Mat H = Mat::eye(3, 3, CV_64FC1);
    if (pts_all.size() > 50) {
        std::vector<unsigned char> match_mask;
        Mat temp = findHomography(prev_pts_all, pts_all, RANSAC, 1, match_mask);
        if (countNonZero(Mat(match_mask)) > 25)
            H = temp;
    }
    
    Mat H_inv = H.inv();
    Mat grey_warp = Mat::zeros(grey.size(), CV_8UC1);
    desc.MyWarpPerspective(prev_grey, grey, grey_warp, H_inv, CV_INTER_LINEAR, cv::BORDER_CONSTANT, Scalar());
    
    // compute optical flow for all scales once
    optFlow.FarnebackPolyExpPyr(grey_warp, poly_warp_pyr, fscales, 7, 1.5);
    optFlow.calcOpticalFlowFarneback(prev_poly_pyr, poly_warp_pyr, flow_warp_pyr, 10, 2);
    
    // Processing for each scale image
    for (int iScale = 0; iScale < param->scale_num; iScale++) {
        if (iScale == 0)
            grey.copyTo(grey_pyr[0]);
        else
            resize(grey_pyr[iScale - 1], grey_pyr[iScale], grey_pyr[iScale].size(), 0, 0, INTER_LINEAR);
        
        int width = grey_pyr[iScale].cols;
        int height = grey_pyr[iScale].rows;
        
        // compute the integral histograms
        ParamManager::DescMat *hogMat = init.InitDescMat(height + 1, width + 1, hogInfo.nBins);
        desc.HogComp(prev_grey_pyr[iScale], hogMat->desc, hogInfo);
        
        ParamManager::DescMat *hofMat = init.InitDescMat(height + 1, width + 1, hofInfo.nBins);
        desc.HofComp(flow_warp_pyr[iScale], hofMat->desc, hofInfo);
        
        ParamManager::DescMat *mbhMatX = init.InitDescMat(height + 1, width + 1, mbhInfo.nBins);
        ParamManager::DescMat *mbhMatY = init.InitDescMat(height + 1, width + 1, mbhInfo.nBins);
        desc.MbhComp(flow_warp_pyr[iScale], mbhMatX->desc, mbhMatY->desc, mbhInfo);
        
        std::list<Track> &tracks = xyScaleTracks[iScale];
        for (std::list<Track>::iterator iTrack = tracks.begin(); iTrack != tracks.end();) {
            int index = iTrack->index;
            Point2f prev_point = iTrack->point[index];
            int x = std::min<int>(std::max<int>(cvRound(prev_point.x), 0), width - 1);
            int y = std::min<int>(std::max<int>(cvRound(prev_point.y), 0), height - 1);
            
            // Set the tracking feature point
            Point2f point;
            point.x = prev_point.x + flow_pyr[iScale].ptr<float>(y)[2 * x];
            point.y = prev_point.y + flow_pyr[iScale].ptr<float>(y)[2 * x + 1];
            
            if (point.x <= 0 || point.x >= width || point.y <= 0 || point.y >= height) {
                iTrack = tracks.erase(iTrack);
                continue;
            }
            
            iTrack->disp[index].x = flow_warp_pyr[iScale].ptr<float>(y)[2 * x];
            iTrack->disp[index].y = flow_warp_pyr[iScale].ptr<float>(y)[2 * x + 1];
            
            // Get the rectangle
            ParamManager::RectInfo rect;
            desc.GetRect(prev_point, rect, width, height, hogInfo);
            
            // Get the descriptirs for the feature point
            desc.GetDesc(hogMat, rect, hogInfo, iTrack->hog, index);
            desc.GetDesc(hofMat, rect, hofInfo, iTrack->hof, index);
            desc.GetDesc(mbhMatX, rect, mbhInfo, iTrack->mbhX, index);
            desc.GetDesc(mbhMatY, rect, mbhInfo, iTrack->mbhY, index);
            
            // Addition the trcking point
            iTrack->addPoint(point);
            
            //for draw track
            if(iScale == 0)
                desc.DrawTrack(iTrack->point, iTrack->index, fscales[iScale], image);
            
            // if the trajectory achieves the maximal length
            if (iTrack->index >= trackInfo.length) {
                std::vector<Point2f> trajectory(trackInfo.length + 1);
                for (int i = 0; i <= trackInfo.length; ++i)
                    trajectory[i] = iTrack->point[i] * fscales[iScale];
                
                std::vector<Point2f> displacement(trackInfo.length);
                for (int i = 0; i < trackInfo.length; ++i)
                    displacement[i] = iTrack->disp[i] * fscales[iScale];
                
                float mean_x(0);
                float mean_y(0);
                float var_x(0);
                float var_y(0);
                float length(0);
                
                if (desc.IsValid(trajectory, mean_x, mean_y, var_x, var_y, length) && desc.IsCameraMotion(displacement)) {
                    desc.StoreDesc(iTrack->hog, hogInfo, trackInfo, Store_HOG);
                    desc.StoreDesc(iTrack->hof, hofInfo, trackInfo, Store_HOF);
                    desc.StoreDesc(iTrack->mbhX, mbhInfo, trackInfo, Store_MBHx);
                    desc.StoreDesc(iTrack->mbhY, mbhInfo, trackInfo, Store_MBHy);
                }
                
                iTrack = tracks.erase(iTrack);
                continue;
            }
            ++iTrack;
        }
        
        init.ReleDescMat(hogMat);
        init.ReleDescMat(hofMat);
        init.ReleDescMat(mbhMatX);
        init.ReleDescMat(mbhMatY);
        
        if (init_counter != trackInfo.gap)
            continue;
        
        // detect new feature points every gap frames
        std::vector<Point2f> points(0);
        for (std::list<Track>::iterator iTrack = tracks.begin(); iTrack != tracks.end(); iTrack++)
            points.push_back(iTrack->point[iTrack->index]);
        
        // Extract new feature points
        desc.DenseSample(grey_pyr[iScale], points, param->quality, param->min_distance);
        
        // save the new feature points
        for (int i = 0; i < points.size(); i++)
            tracks.push_back(Track(points[i], trackInfo, hogInfo, hofInfo, mbhInfo));
    }
    
    init_counter = 0;
    grey.copyTo(prev_grey);
    for (int i = 0; i < param->scale_num; i++) {
        grey_pyr[i].copyTo(prev_grey_pyr[i]);
        poly_pyr[i].copyTo(prev_poly_pyr[i]);
    }
    
    prev_kpts_surf = kpts_surf;
    desc_surf.copyTo(prev_desc_surf);
    
    frame_num++;
}

void ofxDenseTrajectories::drawTrajectories(){
    if(image.empty())
        return;

//    cvtColor(image, image, CV_RGB2BGR);
    ofxCv::drawMat(image, 0, 0, seqInfo.width/2, seqInfo.height/2);
}

void ofxDenseTrajectories::writeCSV(std::string csvPath, DescType type) {
    std::ofstream write;
    write.open(csvPath, std::ios::out);

    switch (type) {
        case DescType::HOG:
            for(int i = 0 ; i < Store_HOG.size() ; ++i){
                for(int j = 0 ; j < Store_HOG[i].size() ; ++j){
                    write << Store_HOG[i][j] << ",";
                }
                write << std::endl;
            }
            write.close();
            ofLogNotice() << "Write(HOG) : " << csvPath;
            break;
            
        case DescType::HOF:
            for(int i = 0 ; i < Store_HOF.size() ; ++i){
                for(int j = 0 ; j < Store_HOF[i].size() ; ++j){
                    write << Store_HOF[i][j] << ",";
                }
                write << std::endl;
            }
            write.close();
            ofLogNotice() << "Write(HOF) : " << csvPath;
            break;
            
        case DescType::MBHx:
            for(int i = 0 ; i < Store_MBHx.size() ; ++i){
                for(int j = 0 ; j < Store_MBHx[i].size() ; ++j){
                    write << Store_MBHx[i][j] << ",";
                }
                write << std::endl;
            }
            write.close();
            ofLogNotice() << "Write(MBHx) : " << csvPath;
            break;
            
        case DescType::MBHy:
            for(int i = 0 ; i < Store_MBHy.size() ; ++i){
                for(int j = 0 ; j < Store_MBHy[i].size() ; ++j){
                    write << Store_MBHy[i][j] << ",";
                }
                write << std::endl;
            }
            write.close();
            ofLogNotice() << "Write(MBHy) : " << csvPath;
            break;
            
        default:
            ofLogNotice() << "No Write";
            write.close();
            break;
    }
}

void ofxDenseTrajectories::clearDesc(DescType type){
    switch (type) {
        case DescType::HOG:
            for(int i = 0 ; i < Store_HOG.size() ; ++i)
                Store_HOG.at(i).clear();
            Store_HOG.clear();

            break;
            
        case DescType::HOF:
            for(int i = 0 ; i < Store_HOF.size() ; ++i)
                Store_HOF.at(i).clear();
            Store_HOF.clear();
            break;
            
        case DescType::MBHx:
            for(int i = 0 ; i < Store_MBHx.size() ; ++i)
                Store_MBHx.at(i).clear();
            Store_MBHx.clear();
            break;
            
        case DescType::MBHy:
            for(int i = 0 ; i < Store_MBHy.size() ; ++i)
                Store_MBHy.at(i).clear();
            Store_MBHy.clear();
            break;
            
        default:
            break;
    }
}

