//
// Created by seymour on 24/01/18.
//

#ifndef TERA_PARAMS_HPP
#define TERA_PARAMS_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

#include "Settings.hpp"

#include <GL/glu.h>

class Params {
public:
  /*  Params(cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
           const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
           const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f>>& imagePoints,
           double totalAvgErr ) : cameraMatrix{cameraMatrix},
                                  distCoeffs{distCoeffs}
                                  ,imagePoints{imagePoints}
                                  ,objectPoints{} {}*/

    Params();


    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Mat tempPoints;

    cv::Mat imagePoints;

    cv::Mat objectPoints;


    int width;
    int height;

    int board_Width;
    int board_Height;


    void lookAt();

    void Frutsum();

    void InitWorldPoints(int width, int height, float largeur);

    void Render();

    void UpdateWorldPoints();

    void UpdateWorldPoints(cv::Mat source);

    static bool readStringList( const std::string& filename, vector<std::string>& l )
    {
        l.clear();
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        cv::FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != cv::FileNode::SEQ )
            return false;
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }



    void read(const cv::FileNode& node)                          //Read serialization for this class
    {


        node["Camera_Matrix"] >> cameraMatrix;
        node["Distortion_Coefficients"]   >> distCoeffs;
        node["board_Width"]  >> board_Width;
        node["board_Height"] >> board_Height;
        node["image_Height"] >> height;
        node["image_Width"]  >> width;
        node["Image_points"] >> tempPoints;

        imagePoints = tempPoints.row(0);
    }


    void Mult();
};


#endif //TERA_PARAMS_HPP
