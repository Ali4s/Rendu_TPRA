#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "include/Calibration.hpp"
#include "include/Window.hpp"
#include "include/Params.hpp"


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

static void read(const cv::FileNode& node, Params& x, const Params& default_value = Params())
{
        x.read(node);
}

static void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}
int main() {

    /* Calibration temp;
      Settings s;
      const string inputSettingsFile = "/home/seymour/Documents/TERA/assets/settings.xml";
      cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
      if (!fs.isOpened())
      {
          cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
          return -1;
      }
      fs["Settings"] >> s;
      fs.release();

      temp.MainCalibration(s);*/

     Params p;
     //extraire les infos du fichier
    const std::string inputSettingsFile = "/home/seymour/Documents/TERA/assets/out_camera_data.xml";
     cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
     if (!fs.isOpened()) {
         cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
         return -1;
     }




     fs["opencv_storage"] >> p;

     fs.release();

     //p.InitWorldPoints(7,4,1.5);

     Calibration calib;


     //prendre la vision sur la caméra
     cv::VideoCapture capture( 0);

     if( !capture.isOpened() )
     {
         cout << "Cannot open capture object." << endl;
         exit(-1);
     }

     cv::Mat camFrame;

     if (!capture.grab())
     {
         cout << "Could not grab kinect... Skipping frame." << endl;
     }
     else {
         capture.retrieve(camFrame, CV_CAP_OPENNI_BGR_IMAGE);
     }
     cv::namedWindow("edges",1);


     Window win(p.width,p.height);
     for(;;)
     {
         win.show();
         cv::Mat frame;
         capture >> frame; // get a new frame from camera

         imshow("edges", frame);

         if (!capture.grab())
         {
             cout << "Could not grab kinect... Skipping frame." << endl;
         }
         else {
             capture.retrieve(camFrame, CV_CAP_OPENNI_BGR_IMAGE);

             p.UpdateWorldPoints(camFrame);



                 p.Frutsum();
             p.lookAt();


             //retrouver les points grâce à opencv
             //influencer un objet 3D grâce à ces derniers.


             //appliquer la texture sur la fenetre opengl
             GLuint tex = calib.matToTexture(camFrame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP);

             glBindTexture(GL_TEXTURE_2D, tex);
         }


         if(cv::waitKey(30) >= 0) break;
     }

     capture.release();

     glfwTerminate();


    return 0;
}
