//
// Created by lmz on 18-12-18.
//

#ifndef BINDROBOT_CROSSPOINT_H
#define BINDROBOT_CROSSPOINT_H

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

class CrossPoint {
public:
    std::vector<cv::Point> mcrossPoints;
    std::vector<cv::Vec4i> detect(const cv::Mat& src);
    std::vector<cv::Vec4i> detectLines(const cv::Mat& src);
    std::vector<cv::Point> getCrossPoints(std::vector<cv::Vec4i>& lines, std::vector<double> &vdegree);
    void drawPoints(const cv::Mat& src, const std::vector<cv::Vec4i>& lines);

    CrossPoint();

    void Cij(int i, int j, std::vector<int> &r, int num, std::vector<std::vector<int> > & result);
/**
* @brief 对输入图像进行细化,骨骼化
* @param src为输入图像,用cvThreshold函数处理过的8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
* @param maxIterations限制迭代次数，如果不进行限制，默认为-1，代表不限制迭代次数，直到获得最终结果
* @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
*/
    cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1);

/**
* @brief 对骨骼化图数据进行过滤，实现两个点之间至少隔一个空白像素
* @param thinSrc为输入的骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
*/
    void filterOver(cv::Mat thinSrc);

    /**
* @brief 从过滤后的骨骼化图像中寻找端点和交叉点
* @param thinSrc为输入的过滤后骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
* @param raudis卷积半径，以当前像素点位圆心，在圆范围内判断点是否为端点或交叉点
* @param thresholdMax交叉点阈值，大于这个值为交叉点
* @param thresholdMin端点阈值，小于这个值为端点
* @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
*/
    std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis = 4, unsigned int thresholdMax = 6, unsigned int thresholdMin = 4);
};


#endif //BINDROBOT_CROSSPOINT_H
