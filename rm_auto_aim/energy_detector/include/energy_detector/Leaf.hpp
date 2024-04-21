#ifndef LEAF_HPP_
#define LEAF_HPP_
#include <opencv2/core.hpp>

namespace rm_auto_aim
{
    const int RED = 0;
    const int BLUE = 1;
    const int NONE = 0;
    const int AUTOFIRE = 1;
    const int SMALL = 2;
    const int BIG = 3;
    enum LeafPointType
    {
        TOP_LEFT = 0,
        BOTTOM_LEFT,
        BOTTOM_RIGHT,
        TOP_RIGHT,
        CENTER_POINT
    };
    struct Leaf
    {
        cv::Rect_<float> rect;
        int label;
        float prob;
        bool leaf_type;
        /*
        @param [0]top_left
        @param [1]bottom_left
        @param [2]bottom_right
        @param [3]top_right
        @param [4]center_point
        */
        std::vector<cv::Point2f> kpt;
    };
    
}
#endif