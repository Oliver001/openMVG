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

using namespace openMVG::sfm;

int main(int argc, char **argv) {
  CmdLine cmd;
  std::string sSfM_Data_Filename;

  cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file] path to a SfM_Data scene\n"
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

  /***************************************************/

  // (1)输出有效的、可以进行三角测量的索引号,让
  // 用户选择，并输入想要测量的张数，和每张的索引号
  int drawLinePicNum; ////用户想进行提取的图像的数量
  vector<int> drawLinePicIndex;  //进行直线提取的图片编号
  vector<cv::Point2d> pointPair; // 每个小图左上角的坐标 
#ifdef _WIN32
  std::cout << "有效的索引号:";
  for (size_t i = 0; i < posesIndex.size(); i++)
    cout << posesIndex[i] << " ";
  std::cout << endl;
  std::cout << "输入要匹配的张数";
  
  std::cin >> drawLinePicNum;
  std::cout << "输入每张的索引号(每张的索引号，不能一样)";
  for (int i = 0; i < drawLinePicNum; i++) {
    int tmp;
    std::cin >> tmp;
    drawLinePicIndex.push_back(tmp);
  }
#else
    // 读取图片的总数目和id
  std::fstream idx_id_in(my_sfm_data.s_root_path + "/../matches/idx_id.txt", std::ios::in);
  idx_id_in >> drawLinePicNum;
  int temp = 0;
  for (int i = 0; i < drawLinePicNum; i++) {
    idx_id_in >> temp;
    drawLinePicIndex.push_back(temp);
  }
  idx_id_in.close();
#endif //_WIN32
  //(2)处理每一张图像 //框图 
  for (auto i : drawLinePicIndex) {
    View *view = my_sfm_data.views.at(i).get();
    std::string imgName = my_sfm_data.s_root_path + "/" + view->s_Img_path;

    // 从大图片框出天线，将小图片写到 image_path 同时将小
    // 图片的左上角坐标写到matches目录下image_path.txt
#ifdef _WIN32
    image_path = my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(i) + ".jpg";
    capture(imgName);
#else
    std::string image_path = my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(i) + ".jpg";
#endif //_WIN32
    cv::Point pointOne;
    std::fstream in(image_path + ".txt", std::ios::in);
    in >> pointOne.x >> pointOne.y;
    in.close();
    pointPair.push_back(pointOne);
  }

  //输入框选天线图片的共同前缀名，此处是.../matches/picSmall0, 根据框选的的图片编号合成图片名
  //如.../matches/picSmall01.jpg
  //直线检测结果保存在keyLineArray
  //同时将画上直线的图片写到同一个目录， 如.../matches/picSmalee01_with_lines.jpg
  vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//保存直线提取结果
  std::vector<int> drawLinePair;// 保存每个小图片上选定的直线编号
#ifdef _WIN32
  lineDetector(my_sfm_data.s_root_path + "/../matches/picSmall0", drawLinePicIndex, keyLineArray); // 直线提取
  //读取画好直线的小图片 显示用
  for (int i : drawLinePicIndex) {
    cv::Mat image = cv::imread(my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(i) + "_with_lines.jpg");
    if (!image.empty()) {
      string windowName = std::to_string(i);
      cv::namedWindow(windowName, 1);
      cv::imshow(windowName, image);
      cv::waitKey(0);
    }
  }

  //(3)用户输入直线对的数量，并输入对应的编号，“-1”表示此处的直线没有被提取出来。
  std::cout << "输入直线在每张图像上的编号, 若某张图像上此直线没有提取出来，请输入‘-1’:" << endl;
  for (int j = 0; j < drawLinePicNum; j++) {
    int tmp;
    std::cin >> tmp;
    drawLinePair.push_back(tmp);
  }
  cv::destroyAllWindows();
#else
  /*******************        ANDROID         *****************/
  string smallPicPreName = my_sfm_data.s_root_path + "/../matches/picSmall0";
  for (auto i : drawLinePicIndex) {
    string smallPicName = smallPicPreName + to_string(i) + ".jpg";
    string picWithLine = smallPicPreName + to_string(i) + "_with_lines.jpg";
    vector<cv::line_descriptor::KeyLine> lineInOnePic;
    std::fstream in(picWithLine + ".txt", std::ios::in);
    int j = 0;
    while (!in.eof()) {
      cv::line_descriptor::KeyLine keyline;
      in >> keyline.startPointX >> keyline.startPointY
        >> keyline.endPointX >> keyline.endPointY;
      lineInOnePic.push_back(keyline);
    }
    in.close();
    keyLineArray.push_back(lineInOnePic);
  }
  {
    std::fstream pic_in(my_sfm_data.s_root_path + "/../matches/pic.txt", std::ios::in);
    int temp = -1;
    for (int j = 0; j < drawLinePicNum; j++) {
      pic_in >> temp;
      drawLinePair.push_back(temp);
    }
  }
#endif //_WIN32
  //为计算GPS存储的目标在虚拟空间中的点
  std::vector<openMVG::Vec3> points3DForGPS;
  //代表每次计算出来的水平向量，用以计算平均水平角和水平角的标准差
  std::vector<cv::Vec2d> horizontalVecs;
  std::vector<double> verticalAngles;

  mdzz(my_sfm_data, finalrt, drawLinePicIndex, drawLinePair,
    keyLineArray, horizontalVecs, verticalAngles,
    points3DForGPS, pointPair);
  //角度计算 输出 horizontalVecs verticalAngles points3DForGPS
  computeAVG(my_sfm_data, horizontalVecs, verticalAngles);

  /*******************4、目标GPS与海拔的计算***************/
  GPSandHeight(allGPS, posesIndex, my_sfm_data, points3DForGPS);
  ////使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
  return 0;
}