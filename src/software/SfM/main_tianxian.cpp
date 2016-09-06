// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "third_party\cmdLine\cmdLine.h"
#include "openMVG\cameras\cameras.hpp"
#include "openMVG\sfm\sfm.hpp"
#include "openmvg\geometry\rigid_transformation3D_srt.hpp"
#include "openMVG\cameras\Cameras_Common_command_line_helper.hpp"
#include "openMVG\system\timer.hpp"
#include "openMVG\multiview\triangulation_nview.hpp"

#include "opencv2/line_descriptor.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

#include "tx.h"
#include "capture.h"
#include "proj_api.h"
#include "main_tx.h"

#ifdef __linux__
#include<sys/types.h>
#include<sys/stat.h>
#include<unistd.h>
#define mkdir(x) mkdir(x,0777)
#endif // __linux__


using namespace openMVG::sfm;

int main(int argc, char **argv) {
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "Global Structure from Motion:\n"
    << "-----------------------------------------------------------\n"
    << "Open Source implementation of the paper:\n"
    << "\"Global Fusion of Relative Motions for "
    << "Robust, Accurate and Scalable Structure from Motion.\"\n"
    << "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
    << " ICCV 2013." << std::endl
    << "------------------------------------------------------------"
    << std::endl;
  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sOutDir = "";
  int iRotationAveragingMethod = int(ROTATION_AVERAGING_L2);
  int iTranslationAveragingMethod = int(TRANSLATION_AVERAGING_SOFTL1);
  std::string sIntrinsic_refinement_options = "ADJUST_ALL";

  cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
  cmd.add(make_option('m', sMatchesDir, "matchdir"));
  cmd.add(make_option('o', sOutDir, "outdir"));
  cmd.add(make_option('r', iRotationAveragingMethod, "rotationAveraging"));
  cmd.add(make_option('t', iTranslationAveragingMethod, "translationAveraging"));
  cmd.add(make_option('f', sIntrinsic_refinement_options, "refineIntrinsics"));
  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file] path to a SfM_Data scene\n"
      << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
      << "[-o|--outdir] path where the output data will be stored\n"
      << "\n[Optional]\n"
      << "[-r|--rotationAveraging]\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization (default)\n"
      << "[-t|--translationAveraging]:\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
      << "\t 3 -> SoftL1 minimization (default)\n"
      << "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
      << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      << "\t NONE -> intrinsic parameters are held as constant\n"
      << "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
      << "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
      << "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
      << "\t -> NOTE: options can be combined thanks to '|'\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      << "\t\t-> refine the focal length & the principal point position\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      << "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
      << "\t   \n"
      << "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
      << std::endl;
    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }




  /*********************1、旋转转化矩阵的计算********************/
  //(1)获取SFM DATA
  SfM_Data my_sfm_data;
  if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }



  //需要创建的四个文件夹
  string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
  mkdir(txtPath.c_str());

  string points3DPath = my_sfm_data.s_root_path + "/../" + "points3D";
  mkdir(points3DPath.c_str());

  string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
  mkdir(GPSPath.c_str());

  mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());


  //(2)获取虚拟空间的旋转矩阵
  size_t viewsNum = my_sfm_data.views.size();
  size_t posesNum = my_sfm_data.poses.size();
  vector<Eigen::Matrix<double, 3, 3>> rotations;
  vector<int> posesIndex;
  for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
    rotations.push_back(itr->second.rotation().inverse());
    //存储有效poses的索引号
    posesIndex.push_back(itr->first);
  }

  //(3)获取现实空间中的旋转矩阵
  //(4)将手机矩阵转化为相机矩阵Remap

  string filePath = my_sfm_data.s_root_path;
  vector<Eigen::Matrix<double, 3, 3> > rotationsAndroid;
  vector<vector<double> > allGPS; 
  readRotations(my_sfm_data, posesIndex, rotationsAndroid, allGPS, filePath);
  Eigen::Matrix<double, 3, 3> finalrt;
  virtualToReality(finalrt, posesNum, rotationsAndroid, rotations, my_sfm_data.s_root_path + "/..");


  /**********************************************************************************************************/
  /*************************         ANDROID           **************************************/
  // (1)输出有效的、可以进行三角测量的索引号,让
  // 用户选择，并输入想要测量的张数，和每张的索引号

  int drawLinePicNum; ////用户想进行提取的图像的数量
  vector<int> drawLinePicIndex;  //进行直线提取的图片编号

  std::cout << "有效的索引号:";
  for (size_t i = 0; i < posesIndex.size(); i++)
    cout << posesIndex[i] << " ";
  std::cout << endl;
  std::cout << "输入要匹配的张数：";
  
  std::cin >> drawLinePicNum;
  std::cout << "输入每张的索引号(每张的索引号，不能一样)：";
  for (int i = 0; i < drawLinePicNum; i++) {
    int tmp;
    std::cin >> tmp;
    drawLinePicIndex.push_back(tmp);
  }
 
  //(2)处理每一张图像
  //框图 
  vector<cv::Point2d> pointPair; // 每个小图左上角的坐标 
  for (auto i : drawLinePicIndex)
  {
	  View *view = my_sfm_data.views.at(i).get();
	  string imgName = my_sfm_data.s_root_path + "/" + view->s_Img_path;
	  image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
	  capture(imgName); //从大图片框出天线，将小图片写到 image_path 同时将小图片的左上角坐标写到image_path.txt
						//在matches目录下
	  cv::Point pointOne;
	  std::fstream in(image_path + ".txt", std::ios::in);
	  in >> pointOne.x >> pointOne.y;
	  in.close();
	  pointPair.push_back(pointOne);
  }
  /***********************************************************************************/
  /*****************************************************************************************************************/



  //输入框选天线图片的共同前缀名，此处是.../matches/picSmall0, 根据框选的的图片编号合成图片名
  //如.../matches/picSmall01.jpg
  //直线检测结果保存在keyLineArray
  //同时将画上直线的图片写到同一个目录， 如.../matches/picSmalee01_with_lines.jpg

  vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//保存直线提取结果
  lineDetector(my_sfm_data.s_root_path + "/../" + "matches/picSmall0", drawLinePicIndex, keyLineArray); // 直线提取


  /*******************                   ANDROID                       *****************/
  //读取画好直线的小图片 显示用
  for (int i : drawLinePicIndex)
  {
	  cv::Mat image = cv::imread(my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + "_with_lines.jpg");
	  if (!image.empty())
	  {
		  string windowName = std::to_string(i);
		  cv::namedWindow(windowName, 1);
		  cv::imshow(windowName,image);
		  cv::waitKey(0);
	  }
  }


  //(3)用户输入直线对的数量，并输入对应的编号，“-1”表示此处的直线没有被提取出来。

  std::vector<int> drawLinePair;// 保存每个小图片上选定的直线编号
  std::cout << "输入直线在每张图像上的编号, 若某张图像上此直线没有提取出来，请输入‘-1’:" << endl;

  int validNumOfImgs = drawLinePicNum;//所有进行了直线提取的图片数量，

  for (int j = 0; j < drawLinePicNum; j++) {
    int tmp;
    std::cin >> tmp;
    //若输入-1，则有效图像数减1
    if (tmp == -1) {
      validNumOfImgs--;
    }
    drawLinePair.push_back(tmp);
  }
  cv::destroyAllWindows();
/**************************************************************************************/



  //为计算GPS存储的目标在虚拟空间中的点
  std::vector<openMVG::Vec3> points3DForGPS;
  //代表每次计算出来的水平向量，用以计算平均水平角和水平角的标准差
  std::vector<cv::Vec2d> horizontalVecs;
  std::vector<double> verticalAngles;
 
  mdzz(my_sfm_data, finalrt, drawLinePicIndex, drawLinePair,
	  keyLineArray, horizontalVecs, verticalAngles,
	  points3DForGPS, pointPair);                           //角度计算 输出 horizontalVecs verticalAngles points3DForGPS

  computeAVG(my_sfm_data, horizontalVecs, verticalAngles);

  /*******************4、目标GPS与海拔的计算***************/

  GPSandHeight(allGPS, posesIndex, my_sfm_data, points3DForGPS);
  
  
  ////使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
  
}