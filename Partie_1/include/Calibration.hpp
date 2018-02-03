//
// Created by seymour on 17/01/18.
//

#ifndef TERA_CALIBRATION_HPP
#define TERA_CALIBRATION_HPP


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include "GLFW/glfw3.h"

#include "Settings.hpp"


class Calibration {

    static double computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                            const vector<vector<cv::Point2f>> &imagePoints, const vector<cv::Mat> &rvecs,
                                            const vector<cv::Mat> &tvecs, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                            vector<float> &perViewErrors);


    static bool runCalibration(Settings &s, cv::Size &imageSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                        vector<vector<cv::Point2f>> imagePoints, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs,
                        vector<float> &reprojErrs, double &totalAvgErr);

    static bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat&  cameraMatrix, cv::Mat& distCoeffs,
                                            vector<vector<cv::Point2f> > imagePoints );

    static void saveCameraParams(Settings &s, cv::Size &imageSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                          const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs, const vector<float> &reprojErrs,
                          const vector<vector<cv::Point2f>> &imagePoints, double totalAvgErr);



public:
    static int MainCalibration(Settings s);
    static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, vector<cv::Point3f> &corners,
                                         Settings::Pattern patternType);

    static GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);
    static int    frameCount;

    static void draw(cv::Mat &camFrame, cv::Mat &depthFrame);
};


#endif //TERA_CALIBRATION_HPP
