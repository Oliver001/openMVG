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




  /*********************1����תת������ļ���********************/
  //(1)��ȡSFM DATA
  SfM_Data my_sfm_data;
  if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }



  //��Ҫ�������ĸ��ļ���
  string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
  mkdir(txtPath.c_str());

  string points3DPath = my_sfm_data.s_root_path + "/../" + "points3D";
  mkdir(points3DPath.c_str());

  string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
  mkdir(GPSPath.c_str());

  mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());


  //(2)��ȡ����ռ����ת����
  size_t viewsNum = my_sfm_data.views.size();
  size_t posesNum = my_sfm_data.poses.size();
  vector<Eigen::Matrix<double, 3, 3>> rotations;
  vector<int> posesIndex;
  for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
    rotations.push_back(itr->second.rotation().inverse());
    //�洢��Чposes��������
    posesIndex.push_back(itr->first);
  }

  //(3)��ȡ��ʵ�ռ��е���ת����
  //(4)���ֻ�����ת��Ϊ�������Remap

  string filePath = my_sfm_data.s_root_path;
  vector<Eigen::Matrix<double, 3, 3> > rotationsAndroid;
  vector<vector<double> > allGPS; 
  readRotations(my_sfm_data, posesIndex, rotationsAndroid, allGPS, filePath);
  Eigen::Matrix<double, 3, 3> finalrt;
  virtualToReality(finalrt, posesNum, rotationsAndroid, rotations, my_sfm_data.s_root_path + "/..");


  /**********************************************************************************************************/
  /*************************         ANDROID           **************************************/
  // (1)�����Ч�ġ����Խ������ǲ�����������,��
  // �û�ѡ�񣬲�������Ҫ��������������ÿ�ŵ�������

  int drawLinePicNum; ////�û��������ȡ��ͼ�������
  vector<int> drawLinePicIndex;  //����ֱ����ȡ��ͼƬ���

  std::cout << "��Ч��������:";
  for (size_t i = 0; i < posesIndex.size(); i++)
    cout << posesIndex[i] << " ";
  std::cout << endl;
  std::cout << "����Ҫƥ���������";
  
  std::cin >> drawLinePicNum;
  std::cout << "����ÿ�ŵ�������(ÿ�ŵ������ţ�����һ��)��";
  for (int i = 0; i < drawLinePicNum; i++) {
    int tmp;
    std::cin >> tmp;
    drawLinePicIndex.push_back(tmp);
  }
 
  //(2)����ÿһ��ͼ��
  //��ͼ 
  vector<cv::Point2d> pointPair; // ÿ��Сͼ���Ͻǵ����� 
  for (auto i : drawLinePicIndex)
  {
	  View *view = my_sfm_data.views.at(i).get();
	  string imgName = my_sfm_data.s_root_path + "/" + view->s_Img_path;
	  image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
	  capture(imgName); //�Ӵ�ͼƬ������ߣ���СͼƬд�� image_path ͬʱ��СͼƬ�����Ͻ�����д��image_path.txt
						//��matchesĿ¼��
	  cv::Point pointOne;
	  std::fstream in(image_path + ".txt", std::ios::in);
	  in >> pointOne.x >> pointOne.y;
	  in.close();
	  pointPair.push_back(pointOne);
  }
  /***********************************************************************************/
  /*****************************************************************************************************************/



  //�����ѡ����ͼƬ�Ĺ�ͬǰ׺�����˴���.../matches/picSmall0, ���ݿ�ѡ�ĵ�ͼƬ��źϳ�ͼƬ��
  //��.../matches/picSmall01.jpg
  //ֱ�߼����������keyLineArray
  //ͬʱ������ֱ�ߵ�ͼƬд��ͬһ��Ŀ¼�� ��.../matches/picSmalee01_with_lines.jpg

  vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//����ֱ����ȡ���
  lineDetector(my_sfm_data.s_root_path + "/../" + "matches/picSmall0", drawLinePicIndex, keyLineArray); // ֱ����ȡ


  /*******************                   ANDROID                       *****************/
  //��ȡ����ֱ�ߵ�СͼƬ ��ʾ��
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


  //(3)�û�����ֱ�߶Ե��������������Ӧ�ı�ţ���-1����ʾ�˴���ֱ��û�б���ȡ������

  std::vector<int> drawLinePair;// ����ÿ��СͼƬ��ѡ����ֱ�߱��
  std::cout << "����ֱ����ÿ��ͼ���ϵı��, ��ĳ��ͼ���ϴ�ֱ��û����ȡ�����������롮-1��:" << endl;

  int validNumOfImgs = drawLinePicNum;//���н�����ֱ����ȡ��ͼƬ������

  for (int j = 0; j < drawLinePicNum; j++) {
    int tmp;
    std::cin >> tmp;
    //������-1������Чͼ������1
    if (tmp == -1) {
      validNumOfImgs--;
    }
    drawLinePair.push_back(tmp);
  }
  cv::destroyAllWindows();
/**************************************************************************************/



  //Ϊ����GPS�洢��Ŀ��������ռ��еĵ�
  std::vector<openMVG::Vec3> points3DForGPS;
  //����ÿ�μ��������ˮƽ���������Լ���ƽ��ˮƽ�Ǻ�ˮƽ�ǵı�׼��
  std::vector<cv::Vec2d> horizontalVecs;
  std::vector<double> verticalAngles;
 
  mdzz(my_sfm_data, finalrt, drawLinePicIndex, drawLinePair,
	  keyLineArray, horizontalVecs, verticalAngles,
	  points3DForGPS, pointPair);                           //�Ƕȼ��� ��� horizontalVecs verticalAngles points3DForGPS

  computeAVG(my_sfm_data, horizontalVecs, verticalAngles);

  /*******************4��Ŀ��GPS�뺣�εļ���***************/

  GPSandHeight(allGPS, posesIndex, my_sfm_data, points3DForGPS);
  
  
  ////ʹ����תת�����󣬶������������ƽ�����ת����ΪҪ���������ת��ĵ�����̬����������������У���һ���ǿ�ѡ��
  
}