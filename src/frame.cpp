#include "DynamicObjectsAvoidance/frame.h"

namespace DynamicObjectsAvoidance {

Frame::Frame(long id, int64_t timestamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), timestamp_(timestamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++; // frame id, increase for every new frame
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++; // keyframe id, only increase for new key frame
}

double Frame::findDepth(int y, int x) {
    ushort d = depth_img_.at<uchar>(y, x);
    if ( d > 0.1 ) {
        return double(d);
    } else {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ ) {
            // d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            d = depth_img_.at<uchar>(y+dy[i], x+dx[i]);
            if ( d > 0.1 ) {
                return double(d);
            }
        }
    }
    return -1.0;
}

double Frame::findDepthOnRGBDImage(int y, int x) {
    // 0.001 is the scale transfer depth value from intensity to meter
    double d = depth_img_.at<ushort>(y, x) * 0.001;
    if ( d > 0.1 ) {
        return d;
    } else {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ ) {
            // d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            d = depth_img_.at<ushort>(y+dy[i], x+dx[i]) * 0.001;
            if ( d > 0.1 ) {
                return d;
            }
        }
    }
    return -1.0;
}

}
