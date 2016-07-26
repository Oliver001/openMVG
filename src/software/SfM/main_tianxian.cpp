
// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "tx.h"
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
#include <io.h>

#define MATCHES_DIST_THRESHOLD 25

using namespace openMVG::sfm;
//鼠标画框的相关变量定义
IplImage *src;
IplImage *img;
bool drawing = false;
CvRect rect;
CvPoint origin;
bool istuNumOne = true;
//默认情况下的图像与TXT的路径，可以使用-c进行修改
string image_path = "C:\\Users\\Ethan\\Desktop\\test8m\\";
//默认情况下的图像索引参数，可以在命令行中以-a, -b的形式修改
int firstImgId = 0;
int secondImgId = 1;
//获取鼠标点的位置
void onMouse(int event, int x, int y, int flags, void *param) {
  if (drawing) {
    rect.x = MIN(origin.x, x);
    rect.y = MIN(origin.y, y);
    rect.width = abs(origin.x - x);
    rect.height = abs(origin.y - y);
  }

  if (event == CV_EVENT_LBUTTONDOWN && !CV_EVENT_MOUSEMOVE) {
    drawing = true;
    origin = cvPoint(x, y);
    rect = cvRect(x, y, 0, 0);
  } else if (event == CV_EVENT_LBUTTONUP) {
    drawing = false;
    if (rect.height == 0 || rect.width == 0) {
      //cvDestroyWindow("ScreenShot");
      return;
    }
    img = cvCreateImage(cvSize(rect.width, rect.height), src->depth, src->nChannels);
    cout << rect.x << " " << rect.y << " " << rect.height << " " << rect.width << endl;
    cvSetImageROI(src, rect);
    cvCopy(src, img, 0);
    cvResetImageROI(src);

    //cvNamedWindow("ScreenShot", 1);
    //cvShowImage("ScreenShot", img);
    if (istuNumOne) {
      string filename1 = image_path + "matches\\picSmall01.jpg";
      const char * file1 = filename1.c_str();
      cvSaveImage(file1, img);
      istuNumOne = false;
    } else {
      string filename2 = image_path + "matches\\picSmall02.jpg";
      const char *file2 = filename2.c_str();
      cvSaveImage(file2, img);
      istuNumOne = true;
    }
    return;
  }
}

//比较keyLine
bool sortdes(const cv::line_descriptor::KeyLine &k1, const cv::line_descriptor::KeyLine &k2) {
  return k1.lineLength > k2.lineLength;
}

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
  //第一张图像的索引号以-a表示，第二张图像的索引号以-b表示
  cmd.add(make_option('a', firstImgId, "imgIndex 0"));
  cmd.add(make_option('b', secondImgId, "imgIndex 1"));
  //图像与txt所在的文件夹
  cmd.add(make_option('c', image_path, "image_path"));
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

  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
    iRotationAveragingMethod > ROTATION_AVERAGING_L2) {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  const openMVG::cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
    openMVG::cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
    iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1) {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type) {
    std::cerr << "Invalid: "
      << sImage_describer << " regions type file." << std::endl;
    return EXIT_FAILURE;
  }

  // Features reading
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  // Matches reading
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if // Try to read the two matches file formats
    (
      !(matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.txt")) ||
        matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.bin")))
      ) {
    std::cerr << std::endl
      << "Invalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty()) {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir)) {
    if (!stlplus::folder_create(sOutDir)) {
      std::cerr << "\nCannot create the output directory" << std::endl;
    }
  }

  //---------------------------------------
  // Global SfM reconstruction process
  //---------------------------------------

  openMVG::system::Timer timer;
  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

  // Configure the features_provider & the matches_provider
  sfmEngine.SetFeaturesProvider(feats_provider.get());
  sfmEngine.SetMatchesProvider(matches_provider.get());

  // Configure reconstruction parameters
  sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(
    ERotationAveragingMethod(iRotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(
    ETranslationAveragingMethod(iTranslationAveragingMethod));

  if (sfmEngine.Process()) {
    std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;
    std::cout << "...Generating SfM_Report.html" << std::endl;
    Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));


    /*********************1、旋转转化矩阵的计算********************/
    //(1)获取SFM DATA
    SfM_Data my_sfm_data = sfmEngine.Get_SfM_Data();

    //(2)获取虚拟空间的旋转矩阵
    size_t cameraNum = my_sfm_data.poses.size();
    vector<Eigen::Matrix<double, 3, 3>*> rotations;

    //(3)获取现实空间中的旋转矩阵
    //(4)将手机矩阵转化为相机矩阵Remap
    image_path = image_path + "\\";
    vector<string> txtNames = listTxtFiles(image_path);
    vector<Eigen::Matrix<double, 3, 3>*> rotationsAndroid;

    double tempAngle[3] = { 0.0 };
    Eigen::Matrix<double, 3, 3> rotX,rotZ;
    rotX << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    rotZ << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    for (size_t i = 0; i < cameraNum; i++) {
      Eigen::Matrix<double, 3, 3> tempMat1;
      rotations.push_back(&Eigen::Matrix<double, 3, 3>(my_sfm_data.poses[i].rotation().inverse()));
      getAngleFromTxt(tempAngle, txtNames[i], string("Orientation_NEW_API:"));
      getRotMatrixZXY(tempMat1, -tempAngle[0], tempAngle[2], -tempAngle[1]);
      rotationsAndroid.push_back(&Eigen::Matrix<double, 3, 3>(tempMat1* rotX * rotZ));
    }

  //  {
  //    //(4)将手机矩阵转化为相机矩阵Remap
  //    Eigen::Matrix<double, 3, 3> rotX, rotZ;
  //    double angleX, angleZ;
  //    angleX = (180 * 2 * M_PI) / 360;
  //    angleZ = (-90 * 2 * M_PI) / 360;
  //    rotX << 1, 0, 0, 0, cos(angleX), -sin(angleX), 0, sin(angleX), cos(angleX);
  //    rotZ << cos(angleZ), -sin(angleZ), 0, sin(angleZ), cos(angleZ), 0, 0, 0, 1;
  //    cout << "rotX" << rotX << endl;
  //    cout << "rotY" << rotZ << endl;
  //    Eigen::Matrix<double, 3, 3> tempRot;
  //    for (int i = 0; i < cameraNum; i++) {
  //      rotationsAndroid[i];
  //      *rotationsAndroid[i] * (rotX)* rotZ;
  //      cout << "Remap:" << rotationsAndroid[i] << endl << endl;
  //    }
  //}

    //(5)计算旋转转化矩阵，XYZ, ZXY, ZYX三种分解方式都计算，最后选取了XYZ的方式
    cout << "从虚拟空间旋转至现实空间的旋转矩阵:" << endl;
    double eulerT[3];
    double eulerTotal[3] = { 0.0, 0.0, 0.0 };
    std::vector<double> eulerVector;
    std::vector<Eigen::Matrix<double, 3, 3>> rt(cameraNum);

    cout << endl << "XYZ" << endl;
    for (unsigned int i = 0; i < cameraNum; i++) {
      rt[i] = *rotationsAndroid[i] * (rotations[i]->inverse());
      RotationMatrixToEulerAnglesXYZ(rt[i], eulerT);
    }
    cout << endl << "ZXY" << endl;
    for (unsigned int i = 0; i < cameraNum; i++) {
      RotationMatrixToEulerAnglesZXY(rt[i], eulerT);
    }
    cout << endl << "ZYX" << endl;
    for (unsigned int i = 0; i < cameraNum; i++) {
      RotationMatrixToEulerAnglesZYX(rt[i], eulerT);
      eulerVector.push_back(eulerT[0]);
      eulerVector.push_back(eulerT[1]);
      eulerVector.push_back(eulerT[2]);
    }

    //(6)将旋转矩阵分解成旋转角时，可能分解出+179和-179，这两个量数值差异较大，
    //但是角度上是很接近，所以需要判定并归一化，都化为同一个符号
    //判定是否存在180左右的数值,当一个数值存在大于175，需要检测
    if (abs(eulerVector[0]) > 175) {
      int negativeNum = 0;
      int positiveNum = 0;
      for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
        if (eulerVector[i * 3] > 0) {
          positiveNum++;
        } else {
          negativeNum++;
        }
      }
      if ((positiveNum != eulerVector.size() / 3) || (negativeNum != eulerVector.size() / 3)) {

        if (positiveNum >= negativeNum) {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3] = abs(eulerVector[i * 3]);
          }
        } else {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3] = -abs(eulerVector[i * 3]);
          }
        }
      }
    }

    if (abs(eulerVector[1]) > 175) {
      int negativeNum = 0;
      int positiveNum = 0;
      for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
        if (eulerVector[i * 3 + 1] > 0) {
          positiveNum++;
        } else {
          negativeNum++;
        }
      }
      if ((positiveNum != eulerVector.size() / 3) || (negativeNum != eulerVector.size() / 3)) {
        if (positiveNum >= negativeNum) {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3 + 1] = abs(eulerVector[i * 3 + 1]);
          }
        } else {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3 + 1] = -abs(eulerVector[i * 3 + 1]);
          }
        }
      }

    }

    if (abs(eulerVector[2]) > 175) {
      int negativeNum = 0;
      int positiveNum = 0;
      for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
        if (eulerVector[i * 3 + 2] > 0) {
          positiveNum++;
        } else {
          negativeNum++;
        }
      }
      if ((positiveNum != eulerVector.size() / 3) || (negativeNum != eulerVector.size() / 3)) {
        if (positiveNum >= negativeNum) {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3 + 2] = abs(eulerVector[i * 3 + 2]);
          }
        } else {
          for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
            eulerVector[i * 3 + 2] = -abs(eulerVector[i * 3 + 2]);
          }
        }
      }

    }

    //(7)将旋转转化矩阵分解出的三个旋转角求平均值，并重构成最优的旋转矩阵
    for (unsigned int i = 0; i < eulerVector.size() / 3; i++) {
      eulerTotal[0] += eulerVector[i * 3];
      eulerTotal[1] += eulerVector[i * 3 + 1];
      eulerTotal[2] += eulerVector[i * 3 + 2];
    }

    eulerTotal[0] = eulerTotal[0] / (eulerVector.size() / 3);
    eulerTotal[1] = eulerTotal[1] / (eulerVector.size() / 3);
    eulerTotal[2] = eulerTotal[2] / (eulerVector.size() / 3);
    Eigen::Matrix<double, 3, 3> finalrt;
    getRotMatrixZYX(finalrt, eulerTotal[0], eulerTotal[1], eulerTotal[2]);

    //(8)计算均方值
    double RSME = 0.0;
    for (int i = 0; i < eulerVector.size() / 3; i++) {
      RSME += sqrt((eulerTotal[0] - eulerVector[i * 3 + 0])*(eulerTotal[0] - eulerVector[i * 3 + 0])) +
        +sqrt((eulerTotal[1] - eulerVector[i * 3 + 1])*(eulerTotal[1] - eulerVector[i * 3 + 1]))
        + sqrt((eulerTotal[2] - eulerVector[i * 3 + 2])*(eulerTotal[2] - eulerVector[i * 3 + 2]));
    }
    cout << "Angle Z: " << eulerTotal[0] << " ";
    cout << "Angle Y: " << eulerTotal[1] << " ";
    cout << "Angle X: " << eulerTotal[2] << endl;
    cout << "RSME: " << RSME << endl;
    cout << "Average RSME:" << RSME / eulerVector.size() << endl;

    //(9)创建TXT目录并存储信息
    string txtPath = image_path + "txtFiles";
    mkdir(txtPath.c_str());
    fstream tr0(txtPath + "\\transformRot.txt", ios::out);
    for (int i = 0; i < cameraNum; i++) {
      tr0 << "rt" << i << endl << rt[i] << endl;
    }
    //tr0 << "rt4_2:" << rt4_2 << endl;
    tr0 << "final_rt:" << finalrt << endl;
    tr0.close();

    /*********2、通过直线提取与极限约束获取顶点对以进行三角测量****************/
    openMVG::Triangulation trianObj;

    //(1)获取虚拟空间中第一张图像的view，包含了图像，内参，外参
    View *view = my_sfm_data.views.at(firstImgId).get();
    //获取内外参
    openMVG::cameras::IntrinsicBase * cam = my_sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
    openMVG::geometry::Pose3 pose = my_sfm_data.GetPoseOrDie(view);

    //同理获取第二张图像的view2
    View *view2 = my_sfm_data.views.at(secondImgId).get();
    //获取内外参
    openMVG::cameras::IntrinsicBase * cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
    openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);

    //(2)打开图像并由用户框选天线区域
    //第一张图像
    vector<string> jpgNames;
    jpgNames = listJpgFiles(image_path);
    string img1 = jpgNames[firstImgId];
    src = cvLoadImage(img1.c_str());
    if (!src) {
      printf("Could not load image file: %s\n", img1.c_str());
      return EXIT_FAILURE;
    }
    cvNamedWindow("screenshot", 0);
    //缩小提取的区域
    cvSetMouseCallback("screenshot", onMouse, NULL);//捕捉鼠标
    cvShowImage("screenshot", src);
    cvWaitKey(0);
    cv::Point2d pointOne = CvPoint(rect.x, rect.y);
    cvReleaseImage(&src);
    cvReleaseImage(&img);

    //第二张图像
    string img2 = jpgNames[secondImgId];
    src = cvLoadImage(img2.c_str());
    if (!src) {
      printf("Could not load image file: %s\n", img2.c_str());
      return EXIT_FAILURE;
    }
    cvNamedWindow("screenshot", 0);
    //缩小提取的区域
    cvSetMouseCallback("screenshot", onMouse, NULL);//捕捉鼠标
    cvShowImage("screenshot", src);
    cvWaitKey(0);
    cv::Point2d pointTwo = CvPoint(rect.x, rect.y);
    cvReleaseImage(&src);
    cvReleaseImage(&img);
    cvDestroyAllWindows();

    cout << pointOne << endl;
    cout << pointTwo << endl;

    //保存图像
    string image_path1 = image_path + "matches\\picSmall01.jpg"; //parser.get<String>( 0 );
    string image_path2 = image_path + "matches\\picSmall02.jpg"; //parser.get<String>( 1 );
    if (image_path1.empty() || image_path2.empty()) {
      //help();
      return -1;
    }

    //(3)从路径中读取框选后的小图像，以进行直线提取
    cv::Mat imageMat1 = cv::imread(image_path1, 1);
    cv::Mat imageMat2 = cv::imread(image_path2, 1);

    if (imageMat1.data == NULL || imageMat2.data == NULL) {
      cout << "Error, images could not be loaded. Please, check their path" << endl;
    }

    //直线提取
    /* create binary masks */
    cv::Mat mask1 = cv::Mat::ones(imageMat1.size(), CV_8UC1);
    cv::Mat mask2 = cv::Mat::ones(imageMat2.size(), CV_8UC1);

    /* create a pointer to a BinaryDescriptor object with default parameters */
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

    /* compute lines and descriptors */
    vector<cv::line_descriptor::KeyLine> keylines1, keylines2;
    cv::Mat descr1, descr2;

    (*bd)(imageMat1, mask1, keylines1, descr1, false, false);
    (*bd)(imageMat2, mask2, keylines2, descr2, false, false);

    //(4)计算两张图像的基础矩阵
    //第一种方法，通过两个相机的内外参进行计算
    Eigen::Matrix<double, 3, 4> proj1, proj2;
    openMVG::Vec3 c;
    proj1 = cam->get_projective_equivalent(pose);
    proj2 = cam2->get_projective_equivalent(pose2);
    c = my_sfm_data.poses[firstImgId].center();

    cv::Mat promatric1(3, 4, CV_64FC1);
    cv::Mat promatric2(3, 4, CV_64FC1);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        promatric1.at<double>(i, j) = proj1(i, j);
        promatric2.at<double>(i, j) = proj2(i, j);
      }
    }
    //第一个相机中心
    cv::Mat C(4, 1, CV_64FC1);
    for (int i = 0; i < 3; i++) {
      C.at<double>(i, 0) = c(i, 0);
    }
    C.at<double>(3, 0) = 1.0;

    cv::Mat ee = promatric2*C;

    cv::Mat eInvSym = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
    eInvSym.at<double>(0, 1) = -ee.at<double>(2);
    eInvSym.at<double>(0, 2) = ee.at<double>(1);
    eInvSym.at<double>(1, 0) = ee.at<double>(2);
    eInvSym.at<double>(1, 2) = -ee.at<double>(0);
    eInvSym.at<double>(2, 0) = -ee.at<double>(1);
    eInvSym.at<double>(2, 1) = ee.at<double>(0);

    cv::Mat pro1inv = promatric1.inv(cv::DECOMP_SVD);   //求pro的伪逆矩阵
    cv::Mat FundamentEPP = eInvSym*promatric2*pro1inv;

    //第二种方法，通过调用OPENMVG的接口进行计算
    Eigen::Matrix<double, 3, 3> Fundament = openMVG::F_from_P(cam->get_projective_equivalent(pose), cam->get_projective_equivalent(pose2));
    cv::Mat FundamentFuc(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        FundamentFuc.at<double>(i, j) = Fundament(i, j);
      }
    }
    //输出并比较两种方法的计算结果
    cout << "@@@@@@@@@@" << endl;
    cout << FundamentEPP << endl << FundamentFuc << endl << endl;
    double rular = FundamentEPP.at<double>(0, 0) / FundamentFuc.at<double>(0, 0);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cout << setprecision(15) << FundamentFuc.at<double>(i, j)*rular << endl;
      }
    }
    cout << endl << endl;

    //(5)在两张小图像中，由用户输入图像对
    cout << "分别输入图像1与图像2中对应的两个线段编号:";
    ///////*选择所需线并显示*/
    sort(keylines1.begin(), keylines1.end(), sortdes);
    size_t limitMax = 10 < keylines1.size() ? 10 : keylines1.size();
    for (int i = 0; i < limitMax; i++) {
      line(imageMat1, CvPoint(keylines1[i].startPointX, keylines1[i].startPointY), CvPoint(keylines1[i].endPointX, keylines1[i].endPointY), cv::Scalar(0, 0, 255), 1);
      string putNum = "1";
      putNum[0] = '0' + i;
      cv::Point2d midPoint = CvPoint((keylines1[i].startPointX + keylines1[i].endPointX) / 2, (keylines1[i].startPointY + keylines1[i].endPointY) / 2);
      putText(imageMat1, putNum, midPoint, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
    }
    imshow("第一张图片", imageMat1);


    limitMax = 10 < keylines2.size() ? 10 : keylines2.size();
    sort(keylines2.begin(), keylines2.end(), sortdes);
    for (int i = 0; i < limitMax; i++) {
      line(imageMat2, CvPoint(keylines2[i].startPointX, keylines2[i].startPointY), CvPoint(keylines2[i].endPointX, keylines2[i].endPointY), cv::Scalar(0, 0, 255), 1);
      string putNum = "1";
      putNum[0] = '0' + i;
      cv::Point2d midPoint = CvPoint((keylines2[i].startPointX + keylines2[i].endPointX) / 2, (keylines2[i].startPointY + keylines2[i].endPointY) / 2);
      putText(imageMat2, putNum, midPoint, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
    }
    imshow("第二张图片", imageMat2);

    cv::waitKey(0);

    int lineIdx1 = 0; //线1的代号
    int lineIdx2[10]; //线2的代号
    int lineTp = 0;
    vector<openMVG::Vec2> points_1, points_2;

    fstream out(txtPath + "\\point.txt", ios::out);
    vector<cv::Point2f> point1;
    while (std::cin >> lineIdx1) {
      if (lineIdx1 == -1)
        break;
      cin >> lineIdx2[lineTp++];
      point1.push_back(cv::Point2f(keylines1[lineIdx1].startPointX + pointOne.x, keylines1[lineIdx1].startPointY + pointOne.y));
      point1.push_back(cv::Point2f(keylines1[lineIdx1].endPointX + pointOne.x, keylines1[lineIdx1].endPointY + pointOne.y));
      cout << setprecision(15) << "##" << showpoint << keylines1[lineIdx1].startPointX + pointOne.x << " " << keylines1[lineIdx1].startPointY + pointOne.y << endl;
      cout << setprecision(15) << "##" << showpoint << keylines1[lineIdx1].endPointX + pointOne.x << " " << keylines1[lineIdx1].endPointY + pointOne.y << endl;

      out << setprecision(15) << keylines1[lineIdx1].startPointX + pointOne.x << " " << keylines1[lineIdx1].startPointY + pointOne.y << endl;
      points_1.push_back(openMVG::Vec2(keylines1[lineIdx1].startPointX + pointOne.x, keylines1[lineIdx1].startPointY + pointOne.y));
      out << setprecision(15) << keylines1[lineIdx1].endPointX + pointOne.x << " " << keylines1[lineIdx1].endPointY + pointOne.y << endl;
      points_1.push_back(openMVG::Vec2(keylines1[lineIdx1].endPointX + pointOne.x, keylines1[lineIdx1].endPointY + pointOne.y));
    }
    cv::destroyAllWindows();

    //(6)设置极线图像的路径与图像名称，保留每次计算的极线图像
    //创建路径
    string epipolarImgPath = image_path + "epipolarImg";
    mkdir(epipolarImgPath.c_str());
    //设置图像名称，注意\\仅在数组中占用一位
    string epiImg1Path = "epipolarImg\\fisrtImg00.jpg";
    epiImg1Path[20] = firstImgId + '0';
    epiImg1Path[21] = secondImgId + '0';
    epiImg1Path = image_path + epiImg1Path;
    string epiImg2Path = "epipolarImg\\secondImg00.jpg";
    epiImg2Path[21] = firstImgId + '0';
    epiImg2Path[22] = secondImgId + '0';
    epiImg2Path = image_path + epiImg2Path;

    //(7)画图像1中的两点
    cv::Mat imageMat4 = cv::imread(img1, 1);
    for (int ii = 0; ii < point1.size(); ii++) {
      circle(imageMat4, point1[ii], 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
      if (ii % 2)
        line(imageMat4, point1[ii - 1], point1[ii], cv::Scalar(255, 0, 0), 1);
    }
    imwrite(epiImg1Path, imageMat4);


    //(8)计算极线
    vector<cv::Vec3f> corresEpilines;

    for (int i = 0; i < point1.size(); i++) {
      openMVG::Vec2 tmp(point1[i].x, point1[i].y);
      tmp = cam->get_ud_pixel(tmp);
      cv::Point2f tmp2(tmp.x(), tmp.y());
      point1[i] = tmp2;
    }

    //采用OPENCV的函数计算极线
    computeCorrespondEpilines(point1, 1, FundamentFuc, corresEpilines);

    //另一种计算极线的方法，不使用OPENCV的函数
    //Mat3 FF;
    ////把cv：：mat 转化为Eigen
    //for (int i = 0; i < 3; i++)
    //{
     // for (int j = 0; j < 3; j++)
      //  FF(i, j) = FundamentEPP.at<double>(i, j);
    //}
    //for (int i = 0; i < point1.size(); i++)
    //{
     // const Vec2 l_pt = cam->get_ud_pixel(points_1[i]);
     // const Vec3 line = FF * Vec3(l_pt(0), l_pt(1), 1.);
     // cv::Vec3f coline(line(0), line(1), line(2));
     // corresEpilines.push_back(coline);
    //}

    //(9)画图2中的两点对应的两条极线并计算极线与直线的交点
    cv::Mat imageMat3 = cv::imread(img2, 1);
    lineTp = 0;
    for (vector<cv::Vec3f>::const_iterator it = corresEpilines.begin(); it != corresEpilines.end(); ++it) {
      float a1 = (*it)[0];
      float b1 = (*it)[1];
      float c1 = (*it)[2];
      // draw the epipolar line between first and last column
      line(imageMat3, cv::Point(0.0, -c1 / b1), cv::Point(imageMat3.cols, -(c1 + a1 * imageMat3.cols) / b1), cv::Scalar(0, 255, 0), 1.5);

      //另一条线的a,b,c
      int idx2 = lineIdx2[lineTp / 2];
      lineTp++;
      float a2 = keylines2[idx2].endPointY - keylines2[idx2].startPointY;    //y2-y1
      float b2 = keylines2[idx2].startPointX - keylines2[idx2].endPointX;    //x1-x2
      float c2 = (keylines2[idx2].endPointX + pointTwo.x)*(keylines2[idx2].startPointY + pointTwo.y) - (keylines2[idx2].startPointX + pointTwo.x)*(keylines2[idx2].endPointY + pointTwo.y);    //x2y1-x1y2
                                                                                             //画出竖直线
      if (lineTp % 2 == 0) {
        line(imageMat3, cv::Point(0.0, -c2 / b2), cv::Point(imageMat3.cols, -(c2 + a2 * imageMat3.cols) / b2), cv::Scalar(255, 0, 0), 0.5);
      }

      //计算交点
      cv::Point2d ans;
      ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
      ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
      cout << setprecision(15) << "!!" << showpoint << ans << endl;
      out << setprecision(15) << ans.x << " " << ans.y << endl;
      points_2.push_back(openMVG::Vec2(ans.x, ans.y));
      circle(imageMat3, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
    imwrite(epiImg2Path, imageMat3);

    /**************3、通过顶点对进行三角测量，并计算姿态角**************/
    //(1)三角测量
    std::vector<openMVG::Vec3> points3D;
    openMVG::Vec3 Xtemp;
    for (int i = 0; i < points_1.size(); i++) {
      cout << "first camera's undistorted pixel coordinate:" << endl << cam->get_ud_pixel(points_1[i]) << endl;
      cout << "second camera's undistorted pixel coordinate:" << endl << cam2->get_ud_pixel(points_2[i]) << endl;

      trianObj.add(cam->get_projective_equivalent(pose), cam->get_ud_pixel(points_1[i]));
      trianObj.add(cam2->get_projective_equivalent(pose2), cam2->get_ud_pixel(points_2[i]));
      Xtemp = trianObj.compute();
      //使用旋转转化矩阵，对三角测量后的顶点进行旋转转化，而不是对所有顶点、相机进行转化
      Xtemp = finalrt*Xtemp;
      points3D.push_back(Xtemp);
      trianObj.clear();
    }

    fstream outPoints(txtPath + "\\points3D.txt", ios::out);
    for (int i = 0; i < points3D.size(); i++) {
      outPoints << points3D[i].x() << " " << points3D[i].y() << " " << points3D[i].z() << " " << 255 << " " << 0 << " " << 0 << endl;
    }
    outPoints.close();

    //(2)计算姿态角
    string anglePath = image_path + "angle";
    mkdir(anglePath.c_str());
    string angle = "angle\\angle00.txt";
    angle[11] = firstImgId + '0';
    angle[12] = secondImgId + '0';
    fstream outAngle(image_path + angle, ios::out);
    for (unsigned int i = 0; i < points3D.size(); i = i + 2) {
      double fuyang = getFuYang(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
      double shuiping = getShuiPing(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
      outAngle << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;
    }
    outAngle.close();

    //使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
    for (Landmarks::iterator iterL = my_sfm_data.structure.begin();
      iterL != my_sfm_data.structure.end(); ++iterL) {
      iterL->second.X = finalrt*(iterL->second.X);
    }

    for (Poses::iterator iterP = my_sfm_data.poses.begin();
      iterP != my_sfm_data.poses.end(); ++iterP) {
      openMVG::geometry::Pose3 & pose = iterP->second;
      iterP->second.center() = finalrt*iterP->second.center();
    }

    //-- Export to disk computed scene (data & visualizable results)
    std::cout << "...Export SfM_Data to disk." << std::endl;
    Save(my_sfm_data,
      stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
      ESfM_Data(ALL));

    Save(my_sfm_data,
      stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
      ESfM_Data(ALL));
    return EXIT_SUCCESS;
  }
  return EXIT_SUCCESS;
}
