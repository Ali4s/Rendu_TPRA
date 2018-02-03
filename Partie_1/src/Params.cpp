//
// Created by seymour on 24/01/18.
//

#include "../include/Params.hpp"

/*void Params::InitWorldPoints(int width, int height,float largeur)
{
    if(objectPoints.empty())
    {
        ww = 0;
        for (int i = 0; i < width; ++i) {
            ww++;
            hh=0;
            for (int j = 0; j < height; ++j) {

                cv::Point3f point(largeur*i/10.0,largeur*j/10.0,0.0);
                objectPoints.push_back(point);
                hh++;
            }
        }
    }


}*/


void Params::UpdateWorldPoints(cv::Mat source)
{
    cv::Size patternsize(board_Width,board_Height);

   cv::findChessboardCorners(source,patternsize, objectPoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                                                  + cv::CALIB_CB_FAST_CHECK);
};

void Params::lookAt() {

    cv::Mat outrot,outtrans,resrot,Transpose_rot;

    cv::Point3f x_axis (0.0,.0,50.0);
    cv::Point3f y_axis (0.0,1.0,0.0);

    cv::Mat(x_axis, false);


    cv::solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,outrot,outtrans);
    cv::Rodrigues(outrot,resrot);

    cv::transpose(resrot,Transpose_rot);

    cv::Mat eye = -Transpose_rot * outtrans;

    cv::Mat center = Transpose_rot * (cv::Mat(x_axis, false)- outtrans);

    cv::Mat up = Transpose_rot * cv::Mat(y_axis, false);


    gluLookAt(eye.at<double>(0,0),eye.at<double>(0,1),eye.at<double>(0,2),
              center.at<double>(0,0),center.at<double>(0,1),center.at<double>(0,2),
              up.at<double>(0,0),up.at<double>(0,1),up.at<double>(0,2));


    cv::Mat m;
    cv::Mat mt;

    m = outtrans*resrot;

    cv::transpose(m,mt);


    glMultMatrixf((float *)mt.ptr(0));


}

void Params::Frutsum()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double z_zero = 0.1;
    double far = 100000.0;

    cv::Point3f p (width/2,0.0,1.0);
    cv::Point3f q (0.0,height/2,1.0);
    cv::Point3f r (-width/2,0.0,1.0);
    cv::Point3f s (0.0,-height/2,1.0);

    cv::Mat inverse = cameraMatrix.inv();

    cv::Mat p_bar = cv::Mat(p, false)*cameraMatrix*z_zero;
    cv::Mat q_bar = cv::Mat(q, false)*cameraMatrix*z_zero;
    cv::Mat r_bar = cv::Mat(r, false)*cameraMatrix*z_zero;
    cv::Mat s_bar = cv::Mat(s, false)*cameraMatrix*z_zero;

    double dw = cv::norm(p_bar-r_bar);
    double dh = cv::norm(q_bar-s_bar);

   glFrustum(-dw/2.0,dw/2.0,-dh/2.0,dh/2.0,z_zero,far);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();



}


Params::Params() {


}
