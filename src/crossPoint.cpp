//
// Created by lmz on 18-12-18.
//
#include "crossPoint.h"
#include <unistd.h>

#include <algorithm>

using namespace std;

const int bias = 32;
const int bound = 30;

void CrossPoint::Cij(int i, int j, vector<int> &r, int num, vector<vector<int> > & result)//num = j
{
    //排列组合公式Cij
    //cout << n << ' ' << i << ' ' << j << endl;
    if (j == 1)
    {
        for (int k = 0; k < i; k++)
        {
            vector<int> temp(num);
            r[num - 1] = k;
            for (int i = 0; i < num; i++)
            {
                temp[i] = r[i];
                //cout << r[i] << ' ';
            }
            result.push_back(temp);
            //cout << endl;
        }
    }
    else if (j == 0)
    {
        //do nothing!bound
    }
    else
    {
        for (int k = i; k >= j; k--)
        {
            r[j - 2] = k - 1;
            Cij(k - 1, j - 1, r, num, result);
        }
    }
}
CrossPoint::CrossPoint()
{
    ;
}
std::vector<cv::Vec4i> CrossPoint::detect(const cv::Mat& src) {

    cv::Mat imageConvert;
    src.convertTo(imageConvert, src.type(), 2.5, 0);
    cv::imshow("convertTo image", imageConvert);

    cv::Mat imG;
    cv::cvtColor(imageConvert, imG, CV_BGR2GRAY);

    cv::Mat bw;
    cv::Canny(imG, bw, 150, 120);
    cv::imshow("Canny image", bw);

    cv::Mat se;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));//膨胀大小 实际情况可能需要调整
    dilate(bw, se, element);

    cv::Mat binImage;
    cv::threshold(se, binImage, 127, 1, cv::THRESH_BINARY);

    cv::Mat skel = thinImage(binImage);
    skel *= 255;

    std::vector<cv::Vec4i> lines;

    HoughLinesP(skel, lines, 1, CV_PI / 150, 50, 60, 10);

    //cv::imshow("dilate image", se);
    cv::imshow("skel image", skel);

    return lines;
}
std::vector<cv::Vec4i> CrossPoint::detectLines(const cv::Mat& src) {
    cv::Mat imG;
    cvtColor(src, imG, CV_BGR2GRAY);

    cv::Mat bw;
    cv::Canny(imG, bw, 250, 400);

    cv::Mat se;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));//膨胀大小 实际情况可能需要调整
    dilate(bw, se, element);

    cv::Mat binImage;
    cv::threshold(se, binImage, 127, 1, cv::THRESH_BINARY);

    cv::Mat skel = thinImage(binImage);
    skel *= 255;

    std::vector<cv::Vec4i> lines;
    
    HoughLinesP(skel, lines, 1, CV_PI / 180, 80, 120, 80);

//    cv::imshow("dilate image", se);
    //cv::imshow("skel image", skel);
    
    return lines;
}
std::vector<cv::Point> CrossPoint::getCrossPoints(std::vector<cv::Vec4i>& lines, std::vector<double> &vdegree)
{
    std::vector<cv::Point> beginPoints;
    std::vector<cv::Point> endPoints;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];

        beginPoints.push_back(cv::Point(l[0], l[1]));
        endPoints.push_back(cv::Point(l[2], l[3]));
    }

    mcrossPoints.clear();
    int num = lines.size();
    if(num < 2)
        return mcrossPoints;

    bool bnearPoint(false);
    int count(0);
    for(int i=0; i<lines.size(); i++)
    {
        for(int j=i+1; j<lines.size(); j++)
        {
            double theta1 = atan2(endPoints[i].y-beginPoints[i].y, endPoints[i].x-beginPoints[i].x);
            double theta2 = atan2(endPoints[j].y-beginPoints[j].y, endPoints[j].x-beginPoints[j].x);
            double angle = abs(theta1-theta2) * 180 / 3.14159;

//            cout << i << " " << j << " " << angle << " ";
//            cout<<beginPoints[i].x<<" "<<beginPoints[j].x<<" "<<beginPoints[i].y<<" "<<beginPoints[j].y<<" ";
//            cout<<hypot(beginPoints[i].x-beginPoints[j].x, beginPoints[i].y-beginPoints[j].y)<<endl;
            if( angle<5 || angle>175 && angle<185 || angle>355 )
            {
                double theta3 = atan2(beginPoints[i].y-beginPoints[j].y, beginPoints[i].x-beginPoints[j].x);
                double dist = hypot(beginPoints[i].x-beginPoints[j].x, beginPoints[i].y-beginPoints[j].y);
                double angle2 = abs(theta3-theta1) * 180 / 3.14159;

//                cout<<theta1<<" "<<theta2<<" "<<theta3<<" "<<(abs(theta3-theta1) * 180 / 3.14159)<<endl;
                if( (angle2<5 || angle2>175 && angle2<185 || angle2>355) || dist < 10)
                {
//                    cout<<"delete "<<j<<endl;
                    lines.erase(lines.begin()+j);
                    beginPoints.erase(beginPoints.begin()+j);
                    endPoints.erase(endPoints.begin()+j);
                    j--;
                    continue;
                }
            }
            count++;
            // computer the intersection by two equations of the two lines
            int a1 = beginPoints[i].y -endPoints[i].y;
            int b1 = endPoints[i].x -beginPoints[i].x;
            int c1 = beginPoints[i].x *endPoints[i].y - endPoints[i].x * beginPoints[i].y;

            int a2 = beginPoints[j].y - endPoints[j].y;
            int b2 = endPoints[j].x - beginPoints[j].x;
            int c2 = beginPoints[j].x * endPoints[j].y- endPoints[j].x * beginPoints[j].y;

            int d = a1*b2 - a2*b1;
            if(!d)
                continue;

            int cross_x = (b1*c2 - b2*c1)/d;
            int cross_y = (a2*c1 - a1*c2)/d;

            if(cross_x<bound || cross_x>640-bound || cross_y<bound || cross_y>480-bound)
            {
                continue;
            }
            for(int k=0; k<mcrossPoints.size(); k++)
            {
                if(hypot(mcrossPoints[k].x-cross_x, mcrossPoints[k].y-cross_y)<20)
                {
                    bnearPoint = true;
                    break;
                }
            }
            if(cross_x<0 || cross_x>640 || cross_y<0 || cross_y>480 || bnearPoint)
            {
                bnearPoint = false;
                continue;
            }
            mcrossPoints.push_back(cv::Point(cross_x, cross_y));
            vdegree.push_back(theta1);
        }
    }
    //cout<<"There are "<<lines.size()<<" lines final. "<<endl;

    return mcrossPoints;
}
void CrossPoint::drawPoints(const cv::Mat& src, const std::vector<cv::Vec4i>& lines)
{
    for( size_t i=0; i<lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        //line(src, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(55, 100, 195), 5, cv::LINE_AA);
    }
    //write the image
    static size_t writeI = 0;
    for( size_t i=0; i<mcrossPoints.size(); i++)
    {
        int cross_x = mcrossPoints[i].x;
        int cross_y = mcrossPoints[i].y;
        //circle(src, cv::Point(cross_x, cross_y), 5, cv::Scalar(0, 255, 0), -1);
        
//        if(cross_x-bias>=0 && cross_x+bias<=640 && cross_y-bias>=0 && cross_y+bias<=480)
//        {
//            //cout<<cross_x-bias<<" "<<cross_y-bias<<endl;
//            cv::Rect rect(cross_x-bias, cross_y-bias, 2*bias, 2*bias);
//            cv::Mat image_roi = src(rect);
//
//            stringstream m_sstream;
//
//            string file = "image", result, temp;
//            m_sstream << writeI;
//            m_sstream >> temp;
//            result = file + temp + ".png";
//            m_sstream.clear();
//
//            cv::imwrite( result, image_roi );
//            usleep(50);
//            writeI++;
//        }
    }
    //cv::imshow("Original rgb image", src);
}
cv::Mat CrossPoint::thinImage(const cv::Mat & src, const int maxIterations)
{
    assert(src.type() == CV_8UC1);
    cv::Mat dst;
    int width = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    int count = 0;  //记录迭代次数
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
            break;
        std::vector<uchar *> mFlag; //用于标记需要删除的点

        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                //cout << (int)p1 << endl;
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }

        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
    }
    return dst;
}

void CrossPoint::filterOver(cv::Mat thinSrc)
{
    assert(thinSrc.type() == CV_8UC1);
    int width = thinSrc.cols;
    int height = thinSrc.rows;
    for (int i = 0; i < height; ++i)
    {
        uchar * p = thinSrc.ptr<uchar>(i);
        for (int j = 0; j < width; ++j)
        {
            // 实现两个点之间至少隔一个像素
            //  p9 p2 p3
            //  p8 p1 p4
            //  p7 p6 p5
            uchar p1 = p[j];
            if (p1 != 1) continue;
            uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
            uchar p8 = (j == 0) ? 0 : *(p + j - 1);
            uchar p2 = (i == 0) ? 0 : *(p - thinSrc.step + j);
            uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - thinSrc.step + j + 1);
            uchar p9 = (i == 0 || j == 0) ? 0 : *(p - thinSrc.step + j - 1);
            uchar p6 = (i == height - 1) ? 0 : *(p + thinSrc.step + j);
            uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + thinSrc.step + j + 1);
            uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + thinSrc.step + j - 1);
            if (p2 + p3 + p8 + p9 >= 1)
            {
                p[j] = 0;
            }
        }
    }
}

std::vector<cv::Point> CrossPoint::getPoints(const cv::Mat &thinSrc, unsigned int raudis, unsigned int thresholdMax, unsigned int thresholdMin)
{
    assert(thinSrc.type() == CV_8UC1);
    int width = thinSrc.cols;
    int height = thinSrc.rows;
    cv::Mat tmp;
    thinSrc.copyTo(tmp);
    std::vector<cv::Point> points;
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (*(tmp.data + tmp.step * i + j) == 0)
            {
                continue;
            }
            int count=0;
            for (int k = i - raudis; k < i + raudis+1; k++)
            {
                for (int l = j - raudis; l < j + raudis+1; l++)
                {
                    if (k < 0 || l < 0||k>height-1||l>width-1)
                    {
                        continue;

                    }
                    else if (*(tmp.data + tmp.step * k + l) == 1)
                    {
                        count++;
                    }
                }
            }

            if (count > thresholdMax||count<thresholdMin)
            {
                cv::Point point(j, i);
                points.push_back(point);
            }
        }
    }
    return points;
}
