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
  int firstImgId = 0, secondImgId = 1;

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
    vector<Eigen::Matrix<double, 3, 3>> rotationsAndroid,rotationsAndroidAll;
    Eigen::Matrix<double, 3, 3> rotXZ;
    rotXZ << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    openMVG::sfm::Views views = my_sfm_data.GetViews();
    for (size_t i = 0; i < viewsNum; i++) {
      Eigen::Matrix<double, 3, 3> tempMat1;
      double tempAngle[3] = { 0.0 };
      string filename = my_sfm_data.s_root_path + "/";
      filename = filename + views.at(i)->s_Img_path;
      filename = filename + ".txt";
      string id = "Orientation_NEW_API:";
      getAngleFromTxt(tempAngle, filename, id);
      getRotMatrixZXY(tempMat1, -tempAngle[0], tempAngle[2], -tempAngle[1]);
	  rotationsAndroidAll.push_back(tempMat1 * rotXZ);
    }
	for (size_t i = 0; i != posesIndex.size(); i++) {
		rotationsAndroid.push_back(rotationsAndroidAll[posesIndex[i]]);
	}
    //(5)计算旋转转化矩阵，XYZ, ZXY, ZYX三种分解方式都计算，最后选取了XYZ的方式
    cout << "从虚拟空间旋转至现实空间的旋转矩阵:" << endl;
    std::vector<double> eulerVector;
    std::vector<Eigen::Matrix<double, 3, 3>> rt(posesNum);

    cout << endl << "ZYX" << endl;
    for (int i = 0; i < posesNum; i++) {
      double eulerT[3];
      rt[i] = rotationsAndroid[i] * (rotations[i].inverse());
      RotationMatrixToEulerAnglesZYX(rt[i], eulerT);
      eulerVector.push_back(eulerT[0]);
      eulerVector.push_back(eulerT[1]);
      eulerVector.push_back(eulerT[2]);
    }

    //(6)将旋转矩阵分解成旋转角时，可能分解出+179和-179，这两个量数值差异较大，
    //但是角度上是很接近，所以需要判定并归一化，都化为同一个符号
    //判定是否存在180左右的数值,当一个数值存在大于175，需要检测
    int size_of_eulerVector = posesNum * 3;
    for (int j = 0; j < 3; j++) {
      if (abs(eulerVector[j]) > 175) {
        int positiveNum = 0;
        for (int i = 0; i < size_of_eulerVector; i += 3) {
          positiveNum += (eulerVector[i + j] > 0);
        }
        if (positiveNum < posesNum)
          if (positiveNum < posesNum >> 1) {
            for (int i = 0; i < size_of_eulerVector; i += 3) {
              eulerVector[i + j] = abs(eulerVector[i + j]);
            }
          } else {
            for (int i = 0; i < size_of_eulerVector; i += 3) {
              eulerVector[i + j] = -abs(eulerVector[i + j]);
            }
          }
      }
    }
    //(7)将旋转转化矩阵分解出的三个旋转角求平均值，并重构成最优的旋转矩阵
    double eulerTotal[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < size_of_eulerVector; i += 3) {
      eulerTotal[0] += eulerVector[i];
      eulerTotal[1] += eulerVector[i + 1];
      eulerTotal[2] += eulerVector[i + 2];
    }
    eulerTotal[0] /= posesNum;
    eulerTotal[1] /= posesNum;
    eulerTotal[2] /= posesNum;
    Eigen::Matrix<double, 3, 3> finalrt;
    getRotMatrixZYX(finalrt, eulerTotal[0], eulerTotal[1], eulerTotal[2]);

    //(8)计算均方值
    double RSME = 0.0;
    for (int i = 0; i < eulerVector.size(); i += 3) {
      RSME += (abs(eulerTotal[0] - eulerVector[i]) + abs(eulerTotal[1] - eulerVector[i + 1])
        + abs(eulerTotal[2] - eulerVector[i + 2]));
    }
    cout << "Angle Z: " << eulerTotal[0] << " ";
    cout << "Angle Y: " << eulerTotal[1] << " ";
    cout << "Angle X: " << eulerTotal[2] << endl;
    cout << "RSME: " << RSME << endl;
    cout << "Average RSME:" << RSME / eulerVector.size() << endl;

    //(9)创建TXT目录并存储信息
    string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
    mkdir(txtPath.c_str());
    fstream tr0(txtPath + "/transformRot.txt", ios::out);
    for (int i = 0; i < posesNum; i++) {
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
    string img1Name = my_sfm_data.s_root_path + "/" + view->s_Img_path;
#ifdef _WIN32
    image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall01.jpg";
    capture(img1Name);
#else
    string image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall01.jpg";
#endif // _WIN32
    cv::Point2d pointOne;
    {
      std::fstream in(image_path + ".txt", std::ios::in);
      in >> pointOne.x >> pointOne.y;
      in.close();
    }
    //同理获取第二张图像的view2
    View *view2 = my_sfm_data.views.at(secondImgId).get();
    //获取内外参
    openMVG::cameras::IntrinsicBase * cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
    openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);
    string img2Name = my_sfm_data.s_root_path + "/" + view2->s_Img_path;
#ifdef _WIN32
    image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall02.jpg";
    capture(img2Name);
#else
    /*string */image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall02.jpg";
#endif // _WIN32
    cv::Point2d pointTwo;
    {
      std::fstream in(image_path + ".txt", std::ios::in);
      in >> pointTwo.x >> pointTwo.y;
      in.close();
    }
    cout << pointOne << endl;
    cout << pointTwo << endl;

    //读取框选的图像
    string image_path1 = my_sfm_data.s_root_path + "/../" + "matches/picSmall01.jpg";
    string image_out_path1 = my_sfm_data.s_root_path + "/../" + "matches/picSmall01_with_lines.jpg";
    string image_path2 = my_sfm_data.s_root_path + "/../" + "matches/picSmall02.jpg";
    string image_out_path2 = my_sfm_data.s_root_path + "/../" + "matches/picSmall02_with_lines.jpg";
    if (image_path1.empty() || image_path2.empty()) {
      return EXIT_FAILURE;
    }
#ifdef _WIN32
    if (drawLines(image_path1.c_str(), image_out_path1.c_str()) == EXIT_FAILURE
      || drawLines(image_path2.c_str(), image_out_path2.c_str()) == EXIT_FAILURE) {
      cout << "error occur while get lines in picture" << endl;
      return EXIT_FAILURE;
    }
    cv::Mat imgMat1 = cv::imread(image_out_path1, 1);
    if (imgMat1.data == NULL) {
      cout << "Error, images could not be loaded. Please, check their path" << endl;
      return EXIT_FAILURE;
    }
    cv::Mat imgMat2 = cv::imread(image_out_path2, 1);
    if (imgMat1.data == NULL) {
      cout << "Error, images could not be loaded. Please, check their path" << endl;
      return EXIT_FAILURE;
    }
    imshow("第一张图片", imgMat1);
    imshow("第二张图片", imgMat2);
    cvWaitKey(0);
#endif // !_WIN32
    vector<cv::line_descriptor::KeyLine> keylines1;
    {
      std::fstream in(image_out_path1 + ".txt", std::ios::in);
      float start_x, start_y, end_x, end_y;
      while (in >> start_x && in >> start_y && in >> end_x && in >> end_y) {
        cv::line_descriptor::KeyLine line;
        line.startPointX = start_x;
        line.startPointY = start_y;
        line.endPointX = end_x;
        line.endPointY = end_y;
        keylines1.push_back(line);
        cout << start_x<<" "<<start_y<<" "<<end_x<<" "<<end_y<< endl;
      }
    }
    vector<cv::line_descriptor::KeyLine> keylines2;
    {
      std::fstream in(image_out_path2 + ".txt", std::ios::in);
      float start_x, start_y, end_x, end_y;
      while (in >> start_x && in >> start_y && in >> end_x && in >> end_y) {
        cv::line_descriptor::KeyLine line;
        line.startPointX = start_x;
        line.startPointY = start_y;
        line.endPointX = end_x;
        line.endPointY = end_y;
        keylines2.push_back(line);
        cout << start_x << " " << start_y << " " << end_x << " " << end_y << endl;
      }
    }

    //(4)计算两张图像的基础矩阵
    //第一种方法，通过两个相机的内外参进行计算
    Eigen::Matrix<double, 3, 4> proj1 = cam->get_projective_equivalent(pose),
      proj2 = cam2->get_projective_equivalent(pose2);
    openMVG::Vec3 c = my_sfm_data.poses[firstImgId].center();

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
    cv::Mat FundamentEPP = eInvSym*promatric2*promatric1.inv(cv::DECOMP_SVD);

    //(5)在两张小图像中，由用户输入图像对
    cout << "分别输入图像1与图像2中对应的两个线段编号:";
    int lineIdx1 = 0; //线1的代号
    int lineIdx2[10]; //线2的代号
    int lineTp = 0;
    vector<openMVG::Vec2 > points_1, points_2;
    fstream out(txtPath + "/point.txt", ios::out);
    vector<cv::Point2f> point1;

#ifdef _WIN32
    while (std::cin >> lineIdx1) {
      if (lineIdx1 == -1)
        break;
      cin >> lineIdx2[lineTp++];
      if (lineIdx2[lineTp - 1] == -1)
        break;
#else //__linux__
    fstream picTxtin(my_sfm_data.s_root_path + "/../" + "matches/pic.txt", ios::in);
    while (picTxtin >> lineIdx1) {
      if (lineIdx1 == -1)
        break;
      picTxtin >> lineIdx2[lineTp++];
      if (lineIdx2[lineTp - 1] == -1)
        break;
#endif //_WIN32
      double start_x = keylines1[lineIdx1].startPointX + pointOne.x;
      double start_y = keylines1[lineIdx1].startPointY + pointOne.y;
      double end_x = keylines1[lineIdx1].endPointX + pointOne.x;
      double end_y = keylines1[lineIdx1].endPointY + pointOne.y;
      point1.push_back(cv::Point2f(start_x, start_y));
      point1.push_back(cv::Point2f(end_x, end_y));
      cout << setprecision(15) << "##" << showpoint << start_x << " " << start_y << endl;
      cout << setprecision(15) << "##" << showpoint << end_x << " " << end_y << endl;
      out << setprecision(15) << "##" << showpoint << start_x << " " << start_y << endl;
      out << setprecision(15) << "##" << showpoint << end_x << " " << end_y << endl;
      points_1.push_back(openMVG::Vec2(start_x, start_y));
      points_1.push_back(openMVG::Vec2(end_x, end_y));
    }
#ifdef _WIN32
    cv::destroyAllWindows();
#else //__linux__
    picTxtin.close();
#endif //_WIN32
    //(6)设置极线图像的路径与图像名称，保留每次计算的极线图像
    //创建路径
    mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());
    //设置图像名称，注意\\仅在数组中占用一位
    string epiImg1Path = "epipolarImg/fisrtImg00.jpg";
    epiImg1Path[20] = firstImgId + '0';
    epiImg1Path[21] = secondImgId + '0';
    epiImg1Path = my_sfm_data.s_root_path + "/../" + epiImg1Path;
    string epiImg2Path = "epipolarImg/secondImg00.jpg";
    epiImg2Path[21] = firstImgId + '0';
    epiImg2Path[22] = secondImgId + '0';
    epiImg2Path = my_sfm_data.s_root_path + "/../" + epiImg2Path;

    //(7)画图像1中的两点
    cv::Mat imageMat1 = cv::imread(img1Name, 1);
    for (int i = 0; i < point1.size(); i++) {
      circle(imageMat1, point1[i], 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
      if (i % 2)
        line(imageMat1, point1[i - 1], point1[i], cv::Scalar(255, 0, 0), 1);
    }
    imwrite(epiImg1Path, imageMat1);

    //(8)计算极线
    for (int i = 0; i < point1.size(); i++) {
      openMVG::Vec2 tmp = cam->get_ud_pixel(points_1[i]);
      point1[i] = cv::Point2f(tmp.x(), tmp.y());
    }
    //采用OPENCV的函数计算极线
    vector<cv::Vec3f> corresEpilines;
    computeCorrespondEpilines(point1, 1, FundamentEPP, corresEpilines);

    //(9)画图2中的两点对应的两条极线并计算极线与直线的交点
    cv::Mat imageMat2 = cv::imread(img2Name, 1);
    lineTp = 0;
    for (vector<cv::Vec3f>::const_iterator it = corresEpilines.begin(); it != corresEpilines.end(); ++it) {
      float a1 = (*it)[0];
      float b1 = (*it)[1];
      float c1 = (*it)[2];
      // draw the epipolar line between first and last column
      line(imageMat2, cv::Point(0.0, -c1 / b1), cv::Point(imageMat2.cols, -(c1 + a1 * imageMat2.cols) / b1), cv::Scalar(0, 255, 0), 1.5);

      //另一条线的a,b,c
      int idx2 = lineIdx2[lineTp / 2];
      lineTp++;
      float a2 = keylines2[idx2].endPointY - keylines2[idx2].startPointY;    //y2-y1
      float b2 = keylines2[idx2].startPointX - keylines2[idx2].endPointX;    //x1-x2
      float c2 = (keylines2[idx2].endPointX + pointTwo.x)*(keylines2[idx2].startPointY + pointTwo.y) - (keylines2[idx2].startPointX + pointTwo.x)*(keylines2[idx2].endPointY + pointTwo.y);    //x2y1-x1y2
                                                                                             //画出竖直线
      if (lineTp % 2 == 0) {
        line(imageMat2, cv::Point(0.0, -c2 / b2), cv::Point(imageMat2.cols, -(c2 + a2 * imageMat2.cols) / b2), cv::Scalar(255, 0, 0), 0.5);
      }

      //计算交点
      cv::Point2d ans;
      ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
      ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
      cout << setprecision(15) << "!!" << showpoint << ans << endl;
      out << setprecision(15) << ans.x << " " << ans.y << endl;
      points_2.push_back(openMVG::Vec2(ans.x, ans.y));
      circle(imageMat2, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
    imwrite(epiImg2Path, imageMat2);

    /**************3、通过顶点对进行三角测量，并计算姿态角**************/
    //(1)三角测量
    std::vector<openMVG::Vec3> points3D;
    for (int i = 0; i < points_1.size(); i++) {
      cout << "first camera's undistorted pixel coordinate:" << endl << cam->get_ud_pixel(points_1[i]) << endl;
      cout << "second camera's undistorted pixel coordinate:" << endl << cam2->get_ud_pixel(points_2[i]) << endl;
      trianObj.add(cam->get_projective_equivalent(pose), cam->get_ud_pixel(points_1[i]));
      trianObj.add(cam2->get_projective_equivalent(pose2), cam2->get_ud_pixel(points_2[i]));
      //使用旋转转化矩阵，对三角测量后的顶点进行旋转转化，而不是对所有顶点、相机进行转化
      points3D.push_back(finalrt*trianObj.compute());
      trianObj.clear();
    }

    fstream outPoints(txtPath + "/points3D.txt", ios::out);
    for (int i = 0; i < points3D.size(); i++) {
      outPoints << points3D[i].x() << " " << points3D[i].y() << " " << points3D[i].z() << " " << 255 << " " << 0 << " " << 0 << endl;
    }
    outPoints.close();

    //(2)计算姿态角
    string anglePath = my_sfm_data.s_root_path + "/../" + "angle";
    mkdir(anglePath.c_str());
    string angle = "angle/angle00.txt";
    angle[11] = firstImgId + '0';
    angle[12] = secondImgId + '0';
    fstream outAngle(my_sfm_data.s_root_path + "/../" + angle, ios::out);
    for (unsigned int i = 0; i < points3D.size(); i = i + 2) {
      double fuyang = getFuYang(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
      double shuiping = getShuiPing(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
      outAngle << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;
      cout << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;
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
