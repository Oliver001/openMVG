//
//
#include <iostream>
#include <map>
#include "E:/xiangmu_A/software/SfM/tx_measure.h"
#include "E:/xiangmu_A/software/SfM/myIOutility.h"
#include "E:/xiangmu_A/software/SfM/decrepit.h"


//

using std::cin;
using std::cout;
using std::map;
using std::pair;
int main() {
	string a;
	cin >> a;
	int i;
	cin >> i;

	int read_flag;
	cin >> read_flag;
	const std::string path = "C:\\Users\\Administrator\\Desktop\\" + a + "\\" + std::to_string(i) + "\\";
	const std::string d = path+"Global_Reconstruction\\sfm_data.json";
	TX::MkDirectory(path+"Global_Reconstruction");
	map<string, pair<cv::Point2d, cv::Point2d> > *k = nullptr;
	if (read_flag != 0) {
		 k = TX::Capture::d_CaptureLine(d);
	}
	else
	{
		k = TX::Capture::ReadLineFromFile(path + "line.txt");
		TX::Capture::ShowReadLine(path + "epipolarImg\\s", k);
	}

	TX::TxMeasure tx(d, *k);
	tx.d();
	tx.my_go(2);
}
//
//
//#include "third_party\cmdLine\cmdLine.h"
//#include "openMVG\cameras\cameras.hpp"
//#include "openMVG\sfm\sfm.hpp"
//#include "openmvg\geometry\rigid_transformation3D_srt.hpp"
//#include "openMVG\cameras\Cameras_Common_command_line_helper.hpp"
//#include "openMVG\system\timer.hpp"
//#include "openMVG\multiview\triangulation_nview.hpp"
//
//#include "opencv2/line_descriptor.hpp"
//#include "opencv2/core/utility.hpp"
//#include <opencv2/imgproc.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/line_descriptor.hpp>
//#include <cmath>
//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <map>
//#include "tx.h"
//#include "capture.h"
//#include "proj_api.h"
//#include "main_tx.h"
//#ifdef __linux__
//#include<sys/types.h>
//#include<sys/stat.h>
//#include<unistd.h>
//#define mkdir(x) mkdir(x,0777)
//#endif // __linux__
//
//using namespace openMVG::sfm;
//int main(int argc, char **argv) {
//	using namespace std;
//	std::cout << std::endl
//		<< "-----------------------------------------------------------\n"
//		<< "Global Structure from Motion:\n"
//		<< "-----------------------------------------------------------\n"
//		<< "Open Source implementation of the paper:\n"
//		<< "\"Global Fusion of Relative Motions for "
//		<< "Robust, Accurate and Scalable Structure from Motion.\"\n"
//		<< "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
//		<< " ICCV 2013." << std::endl
//		<< "------------------------------------------------------------"
//		<< std::endl;
//	CmdLine cmd;
//	std::string sSfM_Data_Filename;
//	std::string sMatchesDir;
//	std::string sOutDir = "";
//	int iRotationAveragingMethod = int(ROTATION_AVERAGING_L2);
//	int iTranslationAveragingMethod = int(TRANSLATION_AVERAGING_SOFTL1);
//	std::string sIntrinsic_refinement_options = "ADJUST_ALL";
//
//	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
//	cmd.add(make_option('m', sMatchesDir, "matchdir"));
//	cmd.add(make_option('o', sOutDir, "outdir"));
//	cmd.add(make_option('r', iRotationAveragingMethod, "rotationAveraging"));
//	cmd.add(make_option('t', iTranslationAveragingMethod, "translationAveraging"));
//	cmd.add(make_option('f', sIntrinsic_refinement_options, "refineIntrinsics"));
//	try {
//		if (argc == 1) throw std::string("Invalid parameter.");
//		cmd.process(argc, argv);
//	}
//	catch (const std::string& s) {
//		std::cerr << "Usage: " << argv[0] << '\n'
//			<< "[-i|--input_file] path to a SfM_Data scene\n"
//			<< "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
//			<< "[-o|--outdir] path where the output data will be stored\n"
//			<< "\n[Optional]\n"
//			<< "[-r|--rotationAveraging]\n"
//			<< "\t 1 -> L1 minimization\n"
//			<< "\t 2 -> L2 minimization (default)\n"
//			<< "[-t|--translationAveraging]:\n"
//			<< "\t 1 -> L1 minimization\n"
//			<< "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
//			<< "\t 3 -> SoftL1 minimization (default)\n"
//			<< "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
//			<< "\t ADJUST_ALL -> refine all existing parameters (default) \n"
//			<< "\t NONE -> intrinsic parameters are held as constant\n"
//			<< "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
//			<< "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
//			<< "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
//			<< "\t -> NOTE: options can be combined thanks to '|'\n"
//			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
//			<< "\t\t-> refine the focal length & the principal point position\n"
//			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
//			<< "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
//			<< "\t   \n"
//			<< "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
//			<< std::endl;
//		std::cerr << s << std::endl;
//		return EXIT_FAILURE;
//	}
//
//
//
//
//	/*********************1、旋转转化矩阵的计算********************/
//	//(1)获取SFM DATA
//	SfM_Data my_sfm_data;
//	if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
//		std::cerr << std::endl
//			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
//		return EXIT_FAILURE;
//	}
//
//
//
//	//需要创建的四个文件夹
//	string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
//	mkdir(txtPath.c_str());
//
//	string points3DPath = my_sfm_data.s_root_path + "/../" + "points3D";
//	mkdir(points3DPath.c_str());
//
//	string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
//	mkdir(GPSPath.c_str());
//
//	mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());
//
//
//	//(2)获取虚拟空间的旋转矩阵
//	size_t viewsNum = my_sfm_data.views.size();
//	size_t posesNum = my_sfm_data.poses.size();
//	vector<Eigen::Matrix<double, 3, 3>> rotations;
//	vector<int> posesIndex;
//	for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
//		rotations.push_back(itr->second.rotation());
//		//存储有效poses的索引号
//		posesIndex.push_back(itr->first);
//	}
//
//	//(3)获取现实空间中的旋转矩阵
//	//(4)将手机矩阵转化为相机矩阵Remap
//
//	string filePath = my_sfm_data.s_root_path;
//	vector<Eigen::Matrix<double, 3, 3> > rotationsAndroid;
//	vector<vector<double> > allGPS;
//	readRotations(my_sfm_data, posesIndex, rotationsAndroid, allGPS, filePath);
//	Eigen::Matrix<double, 3, 3> finalrt;
//	virtualToReality(finalrt, posesNum, rotationsAndroid, rotations, my_sfm_data.s_root_path + "/..");
//
//
//	/**********************************************************************************************************/
//	/*************************         ANDROID           **************************************/
//	// (1)输出有效的、可以进行三角测量的索引号,让
//	// 用户选择，并输入想要测量的张数，和每张的索引号
//
//	int drawLinePicNum; ////用户想进行提取的图像的数量
//	vector<int> drawLinePicIndex;  //进行直线提取的图片编号
//
//	std::cout << "有效的索引号:";
//	for (size_t i = 0; i < posesIndex.size(); i++)
//		cout << posesIndex[i] << " ";
//	std::cout << endl;
//	std::cout << "输入要匹配的张数：";
//
//	std::cin >> drawLinePicNum;
//	std::cout << "输入每张的索引号(每张的索引号，不能一样)：";
//	for (int i = 0; i < drawLinePicNum; i++) {
//		int tmp;
//		std::cin >> tmp;
//		drawLinePicIndex.push_back(tmp);
//	}
//
//	//(2)处理每一张图像
//	//框图 
//	vector<cv::Point2d> pointPair; // 每个小图左上角的坐标 
//	for (auto i : drawLinePicIndex)
//	{
//		View *view = my_sfm_data.views.at(i).get();
//		string imgName = my_sfm_data.s_root_path + "/" + view->s_Img_path;
//		image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
//		capture(imgName); //从大图片框出天线，将小图片写到 image_path 同时将小图片的左上角坐标写到image_path.txt
//						  //在matches目录下
//		cv::Point pointOne;
//		std::fstream in(image_path + ".txt", std::ios::in);
//		in >> pointOne.x >> pointOne.y;
//		in.close();
//		pointPair.push_back(pointOne);
//	}
//	/***********************************************************************************/
//	/*****************************************************************************************************************/
//
//
//  //
//  //  输入框选天线图片的共同前缀名，此处是.../matches/picSmall0, 根据框选的的图片编号合成图片名
//  //  如.../matches/picSmall01.jpg
//  //  直线检测结果保存在keyLineArray
//  //  同时将画上直线的图片写到同一个目录， 如.../matches/picSmalee01_with_lines.jpg
//
//	vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//保存直线提取结果
//	lineDetector(my_sfm_data.s_root_path + "/../" + "matches/picSmall0", drawLinePicIndex, keyLineArray); // 直线提取
//
//
//	/*******************                   ANDROID                       *****************/
//	//读取画好直线的小图片 显示用
//	for (int i : drawLinePicIndex)
//	{
//		cv::Mat image = cv::imread(my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + "_with_lines.jpg");
//		if (!image.empty())
//		{
//			string windowName = std::to_string(i);
//			cv::namedWindow(windowName);
//			cv::imshow(windowName, image);
//			cv::waitKey(0);
//		}
//	}
//
//
//	//  (3)用户输入直线对的数量，并输入对应的编号，“-1”表示此处的直线没有被提取出来。
//
//	std::vector<int> drawLinePair;// 保存每个小图片上选定的直线编号
//	std::cout << "输入直线在每张图像上的编号, 若某张图像上此直线没有提取出来，请输入‘-1’:" << endl;
//
//	int validNumOfImgs = drawLinePicNum;//所有进行了直线提取的图片数量，
//
//	for (int j = 0; j < drawLinePicNum; j++) {
//		int tmp;
//		std::cin >> tmp;
//		// 若输入-1，则有效图像数减1
//		if (tmp == -1) {
//			validNumOfImgs--;
//		}
//		drawLinePair.push_back(tmp);
//	}
//	cv::destroyAllWindows();
//
///**************************************************************************************/
//  //为计算GPS存储的目标在虚拟空间中的点
//  std::vector<openMVG::Vec3> points3DForGPS;
// // 代表每次计算出来的水平向量，用以计算平均水平角和水平角的标准差
//  std::vector<cv::Vec2d> horizontalVecs;
//  std::vector<double> verticalAngles;
// 
//  mdzz(my_sfm_data, finalrt, drawLinePicIndex, drawLinePair,
//	  keyLineArray, horizontalVecs, verticalAngles,
//	  points3DForGPS, pointPair);                           //角度计算 输出 horizontalVecs verticalAngles points3DForGPS
//
//  computeAVG(my_sfm_data, horizontalVecs, verticalAngles);
//
//  /*******************4、目标GPS与海拔的计算***************/
//
//  GPSandHeight(allGPS, posesIndex, my_sfm_data, points3DForGPS);
//  
//  
//  //使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
//  
//}
////
////
////// Copyright (c) 2012, 2013, 2014 Pierre MOULON.
////
////// This Source Code Form is subject to the terms of the Mozilla Public
////// License, v. 2.0. If a copy of the MPL was not distributed with this
////// file, You can obtain one at http://mozilla.org/MPL/2.0/.
////
////#include "third_party\cmdLine\cmdLine.h"
////#include "openMVG\cameras\cameras.hpp"
////#include "openMVG\sfm\sfm.hpp"
////#include "openmvg\geometry\rigid_transformation3D_srt.hpp"
////#include "openMVG\cameras\Cameras_Common_command_line_helper.hpp"
////#include "openMVG\system\timer.hpp"
////#include "openMVG\multiview\triangulation_nview.hpp"
////
////#include "opencv2/line_descriptor.hpp"
////#include "opencv2/core/utility.hpp"
////#include <opencv2/imgproc.hpp>
////#include <opencv2/features2d.hpp>
////#include <opencv2/highgui.hpp>
////#include <opencv2/calib3d.hpp>
////
////#include <cmath>
////#include <iostream>
////#include <fstream>
////#include <vector>
////
////#include "tx.h"
////#include "capture.h"
////#include "proj_api.h"
////#ifdef __linux__
////#include<sys/types.h>
////#include<sys/stat.h>
////#include<unistd.h>
////#define mkdir(x) mkdir(x,0777)
////#endif // __linux__
////
////
////using namespace openMVG::sfm;
////
////int main(int argc, char **argv) {
////	using namespace std;
////	std::cout << std::endl
////		<< "-----------------------------------------------------------\n"
////		<< "Global Structure from Motion:\n"
////		<< "-----------------------------------------------------------\n"
////		<< "Open Source implementation of the paper:\n"
////		<< "\"Global Fusion of Relative Motions for "
////		<< "Robust, Accurate and Scalable Structure from Motion.\"\n"
////		<< "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
////		<< " ICCV 2013." << std::endl
////		<< "------------------------------------------------------------"
////		<< std::endl;
////	CmdLine cmd;
////	std::string sSfM_Data_Filename;
////	std::string sMatchesDir;
////	std::string sOutDir = "";
////	int iRotationAveragingMethod = int(ROTATION_AVERAGING_L2);
////	int iTranslationAveragingMethod = int(TRANSLATION_AVERAGING_SOFTL1);
////	std::string sIntrinsic_refinement_options = "ADJUST_ALL";
////
////	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
////	cmd.add(make_option('m', sMatchesDir, "matchdir"));
////	cmd.add(make_option('o', sOutDir, "outdir"));
////	cmd.add(make_option('r', iRotationAveragingMethod, "rotationAveraging"));
////	cmd.add(make_option('t', iTranslationAveragingMethod, "translationAveraging"));
////	cmd.add(make_option('f', sIntrinsic_refinement_options, "refineIntrinsics"));
////	try {
////		if (argc == 1) throw std::string("Invalid parameter.");
////		cmd.process(argc, argv);
////	}
////	catch (const std::string& s) {
////		std::cerr << "Usage: " << argv[0] << '\n'
////			<< "[-i|--input_file] path to a SfM_Data scene\n"
////			<< "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
////			<< "[-o|--outdir] path where the output data will be stored\n"
////			<< "\n[Optional]\n"
////			<< "[-r|--rotationAveraging]\n"
////			<< "\t 1 -> L1 minimization\n"
////			<< "\t 2 -> L2 minimization (default)\n"
////			<< "[-t|--translationAveraging]:\n"
////			<< "\t 1 -> L1 minimization\n"
////			<< "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
////			<< "\t 3 -> SoftL1 minimization (default)\n"
////			<< "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
////			<< "\t ADJUST_ALL -> refine all existing parameters (default) \n"
////			<< "\t NONE -> intrinsic parameters are held as constant\n"
////			<< "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
////			<< "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
////			<< "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
////			<< "\t -> NOTE: options can be combined thanks to '|'\n"
////			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
////			<< "\t\t-> refine the focal length & the principal point position\n"
////			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
////			<< "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
////			<< "\t   \n"
////			<< "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
////			<< std::endl;
////		std::cerr << s << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
////		iRotationAveragingMethod > ROTATION_AVERAGING_L2) {
////		std::cerr << "\n Rotation averaging method is invalid" << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	const openMVG::cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
////		openMVG::cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
////
////	if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
////		iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1) {
////		std::cerr << "\n Translation averaging method is invalid" << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	// Load input SfM_Data scene
////	SfM_Data sfm_data;
////	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
////		std::cerr << std::endl
////			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	// Init the regions_type from the image describer file (used for image regions extraction)
////	using namespace openMVG::features;
////	const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
////	std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
////	if (!regions_type) {
////		std::cerr << "Invalid: "
////			<< sImage_describer << " regions type file." << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	// Features reading
////	std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
////	if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
////		std::cerr << std::endl
////			<< "Invalid features." << std::endl;
////		return EXIT_FAILURE;
////	}
////	// Matches reading
////	std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
////	if // Try to read the two matches file formats
////		(
////			!(matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.txt")) ||
////				matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.bin")))
////			) {
////		std::cerr << std::endl
////			<< "Invalid matches file." << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	if (sOutDir.empty()) {
////		std::cerr << "\nIt is an invalid output directory" << std::endl;
////		return EXIT_FAILURE;
////	}
////
////	if (!stlplus::folder_exists(sOutDir)) {
////		if (!stlplus::folder_create(sOutDir)) {
////			std::cerr << "\nCannot create the output directory" << std::endl;
////		}
////	}
////
////	//---------------------------------------
////	// Global SfM reconstruction process
////	//---------------------------------------
////
////	openMVG::system::Timer timer;
////	GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
////		sfm_data,
////		sOutDir,
////		stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));
////
////	// Configure the features_provider & the matches_provider
////	sfmEngine.SetFeaturesProvider(feats_provider.get());
////	sfmEngine.SetMatchesProvider(matches_provider.get());
////
////	// Configure reconstruction parameters
////	sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
////
////	// Configure motion averaging method
////	sfmEngine.SetRotationAveragingMethod(
////		ERotationAveragingMethod(iRotationAveragingMethod));
////	sfmEngine.SetTranslationAveragingMethod(
////		ETranslationAveragingMethod(iTranslationAveragingMethod));
////
////	if (sfmEngine.Process()) {
////		std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;
////		std::cout << "...Generating SfM_Report.html" << std::endl;
////		Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
////			stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));
////
////
////		/*********************1、旋转转化矩阵的计算********************/
////		//(1)获取SFM DATA
////		SfM_Data my_sfm_data = sfmEngine.Get_SfM_Data();
////
////		//(2)获取虚拟空间的旋转矩阵
////		size_t viewsNum = my_sfm_data.views.size();
////		size_t posesNum = my_sfm_data.poses.size();
////		vector<Eigen::Matrix<double, 3, 3>> rotations;
////		vector<int> posesIndex;
////		for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
////			rotations.push_back(itr->second.rotation());
////			//存储有效poses的索引号
////			posesIndex.push_back(itr->first);
////		}
////
////		//(3)获取现实空间中的旋转矩阵
////		//(4)将手机矩阵转化为相机矩阵Remap
////		vector<Eigen::Matrix<double, 3, 3>> rotationsAndroid, rotationsAndroidAll;
////		Eigen::Matrix<double, 3, 3> rotXZ;
////		rotXZ << 0, 1, 0, 1, 0, 0, 0, 0, -1;
////		openMVG::sfm::Views views = my_sfm_data.GetViews();
////		vector<vector<double>> allGPS;
////		for (size_t i = 0; i < viewsNum; i++) {
////			Eigen::Matrix<double, 3, 3> tempMat1;
////			double tempAngle[3] = { 0.0 };
////			double tempGPS[3] = { 0.0 };
////			vector<double> tempGPSvec;
////			string filename = my_sfm_data.s_root_path + "/";
////			filename = filename + views.at(i)->s_Img_path;
////			filename = filename + ".txt";
////			string angleId = "Orientation_NEW_API:";
////			getInfoFromTxt(tempAngle, filename, angleId);
////			string GPSId = "GPS:";
////			getInfoFromTxt(tempGPS, filename, GPSId);
////			tempGPSvec.push_back(tempGPS[0]);
////			tempGPSvec.push_back(tempGPS[1]);
////			tempGPSvec.push_back(tempGPS[2]);
////			allGPS.push_back(tempGPSvec);
////			getRotMatrixZXY(tempMat1, -tempAngle[0], tempAngle[2], -tempAngle[1]);
////			rotationsAndroidAll.push_back(tempMat1 * rotXZ);
////		}
////		for (size_t i = 0; i != posesIndex.size(); i++) {
////			rotationsAndroid.push_back(rotationsAndroidAll[posesIndex[i]]);
////		}
////		//(5)计算旋转转化矩阵，XYZ, ZXY, ZYX三种分解方式都计算，最后选取了ZYX的方式
////		cout << "从虚拟空间旋转至现实空间的旋转矩阵:" << endl;
////		std::vector<double> eulerVector;
////		std::vector<Eigen::Matrix<double, 3, 3>> rt(posesNum);
////
////		cout << endl << "ZYX" << endl;
////		for (size_t i = 0; i < posesNum; i++) {
////			double eulerT[3];
////			rt[i] = rotationsAndroid[i] * (rotations[i].inverse());
////			RotationMatrixToEulerAnglesZYX(rt[i], eulerT);
////			eulerVector.push_back(eulerT[0]);
////			eulerVector.push_back(eulerT[1]);
////			eulerVector.push_back(eulerT[2]);
////		}
////
////		//(6)将旋转矩阵分解成旋转角时，可能分解出+179和-179，这两个量数值差异较大，
////		//但是角度上是很接近，所以需要判定并归一化，都化为同一个符号
////		//判定是否存在180左右的数值,当一个数值存在大于175，需要检测
////		int size_of_eulerVector = posesNum * 3;
////		for (int j = 0; j < 3; j++) {
////			if (abs(eulerVector[j]) > 175) {
////				size_t positiveNum = 0;
////				for (int i = 0; i < size_of_eulerVector; i += 3) {
////					positiveNum += (eulerVector[i + j] > 0);
////				}
////				if (positiveNum < posesNum)
////					if (positiveNum < posesNum >> 1) {
////						for (int i = 0; i < size_of_eulerVector; i += 3) {
////							eulerVector[i + j] = abs(eulerVector[i + j]);
////						}
////					}
////					else {
////						for (int i = 0; i < size_of_eulerVector; i += 3) {
////							eulerVector[i + j] = -abs(eulerVector[i + j]);
////						}
////					}
////			}
////		}
////		//(7)将旋转转化矩阵分解出的三个旋转角求平均值，并重构成最优的旋转矩阵
////		double eulerTotal[3] = { 0.0, 0.0, 0.0 };
////		for (int i = 0; i < size_of_eulerVector; i += 3) {
////			eulerTotal[0] += eulerVector[i];
////			eulerTotal[1] += eulerVector[i + 1];
////			eulerTotal[2] += eulerVector[i + 2];
////		}
////		eulerTotal[0] /= posesNum;
////		eulerTotal[1] /= posesNum;
////		eulerTotal[2] /= posesNum;
////		Eigen::Matrix<double, 3, 3> finalrt;
////		getRotMatrixZYX(finalrt, eulerTotal[0], eulerTotal[1], eulerTotal[2]);
////
////		//(8)计算均方值
////		double RSME = 0.0;
////		for (size_t i = 0; i < eulerVector.size(); i += 3) {
////			RSME += (abs(eulerTotal[0] - eulerVector[i]) + abs(eulerTotal[1] - eulerVector[i + 1])
////				+ abs(eulerTotal[2] - eulerVector[i + 2]));
////		}
////		cout << "Angle Z: " << eulerTotal[0] << " ";
////		cout << "Angle Y: " << eulerTotal[1] << " ";
////		cout << "Angle X: " << eulerTotal[2] << endl;
////		cout << "RSME: " << RSME << endl;
////		cout << "Average RSME:" << RSME / eulerVector.size() << endl;
////
////		//(9)创建TXT目录并存储信息
////		string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
////		mkdir(txtPath.c_str());
////		fstream tr0(txtPath + "/transformRot.txt", ios::out);
////		for (size_t i = 0; i < posesNum; i++) {
////			tr0 << "rt" << i << endl << rt[i] << endl;
////		}
////		//tr0 << "rt4_2:" << rt4_2 << endl;
////		tr0 << "final_rt:" << finalrt << endl;
////		tr0.close();
////
////
////
////		/*********2、通过直线提取与极限约束获取顶点对以进行三角测量****************/
////
////
////		//(1)输出有效的、可以进行三角测量的索引号,让用户选择，并输入想要测量的张数，和每张的索引号
////		std::cout << "有效的索引号:";
////		for (size_t i = 0; i < posesIndex.size(); i++)
////			cout << posesIndex[i] << " ";
////		std::cout << endl;
////		int drawLinePicNum;                   //用户想进行提取的图像的数量
////		vector<int> drawLinePicIndex;         //要进行提取图像的索引号
////		vector<cv::Point2d> pointPair;
////		std::cout << "输入要匹配的张数：";
////		std::cin >> drawLinePicNum;
////		std::cout << "输入每张的索引号(每张的索引号，不能一样)：";
////		for (int i = 0; i<drawLinePicNum; i++)
////		{
////			int tmp;
////			std::cin >> tmp;
////			drawLinePicIndex.push_back(tmp);
////		}
////
////
////		//(2)处理每一张图像
////		vector<vector<cv::line_descriptor::KeyLine>> keyLineArray;
////		for (int i = 0; i<drawLinePicNum; i++)
////		{
////			//获取虚拟空间中每一张图像的view，包含了图像，内参，外参
////			View *view = my_sfm_data.views.at(drawLinePicIndex[i]).get();
////			//获取内外参
////			openMVG::cameras::IntrinsicBase * cam = my_sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
////			openMVG::geometry::Pose3 pose = my_sfm_data.GetPoseOrDie(view);
////			string img1Name = my_sfm_data.s_root_path + "/" + view->s_Img_path;
////			//cout << view->s_Img_path;
////#ifdef _WIN32
////			image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
////			capture(img1Name);
////#else
////			image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
////			image_path = image_path + to_string(i) + ".jpg";
////#endif // _WIN32
////			cv::Point2d pointOne;
////			{
////				std::fstream in(image_path + ".txt", std::ios::in);
////				in >> pointOne.x >> pointOne.y;
////				in.close();
////			}
////			pointPair.push_back(pointOne);
////
////#ifdef _WIN32
////			//针对每幅图形进行直线提取，并把相关直线保存在drawLineKL
////			//image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0"+to_string(i) + ".jpg";
////			string image_out_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + "_with_lines.jpg";
////			if (image_path.empty()) {
////				return EXIT_FAILURE;
////			}
////			vector<cv::line_descriptor::KeyLine> keylines;
////			keylines = drawLines(image_path.c_str(), image_out_path.c_str());
////			if (keylines.empty()) {
////				cout << "error occur while get lines in picture" << endl;
////				return EXIT_FAILURE;
////			}
////			keyLineArray.push_back(keylines);
////			//显示图像
////			cv::Mat imgMat = cv::imread(image_out_path, 1);
////			if (imgMat.data == NULL) {
////				cout << "Error, images could not be loaded. Please, check their path" << endl;
////				return EXIT_FAILURE;
////			}
////			string windowName = to_string(i) + "图片";
////			imshow(windowName, imgMat);
////
////#endif // !_WIN32
////		}
////		cvWaitKey(0);
////
////
////		//(3)用户输入直线对的数量，并输入对应的编号，“-1”表示此处的直线没有被提取出来。
////#ifdef _WIN32
////		int drawLineNum = 1;
////		std::vector<std::vector<int>> drawLineSet;
////		//std::cout << "输入想测量的直线的个数：";
////		//std::cin >> drawLineNum;
////
////		std::cout << "输入直线在每张图像上的编号, 若某张图像上此直线没有提取出来，请输入‘-1’:" << endl;
////		//如果某图像上此直线没有提取出来，请输入‘-1’
////		//能够提取出直线的图像张数
////		int validNumOfImgs = drawLinePicNum;
////		for (int i = 0; i < drawLineNum; i++)
////		{
////			std::vector<int> drawLinePair;
////			for (int j = 0; j < drawLinePicNum; j++)
////			{
////				int tmp;
////				std::cin >> tmp;
////				//若输入-1，则有效图像数减1
////				if (tmp == -1) {
////					validNumOfImgs--;
////				}
////				drawLinePair.push_back(tmp);
////			}
////			drawLineSet.push_back(drawLinePair);
////		}
////#else //__linux__
////		////////fstream picTxtin(my_sfm_data.s_root_path + "/../" + "matches/pic.txt", ios::in);
////		////////while (picTxtin >> lineIdx1) {
////		////////	if (lineIdx1 == -1)
////		////////		break;
////		////////	picTxtin >> lineIdx2[lineTp++];
////		////////	if (lineIdx2[lineTp - 1] == -1)
////		////////		break;
////#endif //_WIN32
////
////
////#ifdef _WIN32
////		cv::destroyAllWindows();
////#else //__linux__
////		////////picTxtin.close();
////#endif //_WIN32
////
////
////		/**************3、计算两两图像的顶点对，然后进行三角测量，并计算姿态角**************/
////		//此过程可以一次性选择多张图像，并进行两两图像之间的交汇测量，最终求得的是平均值
////		//创建存储极线的路径
////		mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());
////
////		//俯仰角和水平角的平均值
////		//vector<double> averShuiPin(drawLineNum);
////		//vector<double> averFuYang(drawLineNum);
////		//for (int i = 0; i < drawLineNum; i++)
////		//{
////		//	averFuYang[i] = 0.0;
////		//	averShuiPin[i] = 0.0;
////		//}
////		//为计算GPS存储的目标在虚拟空间中的点
////		std::vector<openMVG::Vec3> points3DForGPS;
////		int tuIndex1, tuIndex2;      //两个图像的索引号
////									 //代表每次计算出来的水平向量，用以计算平均水平角和水平角的标准差
////		std::vector<cv::Vec2d> horizontalVecs;
////		std::vector<double> verticalAngles;
////		for (int idi = 0; idi < drawLinePicNum; idi++)
////		{
////			for (int idj = 0; idj < drawLinePicNum; idj++)
////			{
////				//(1)获得图像的索引号，要是是一样图像，不做处理
////				if (idi == idj)
////					continue;
////				bool flag = false;
////				//若有-1，则跳出该图像计算，此处对不同直线没有区别对待
////				for (int k = 0; k < drawLineNum; k++) {
////					if (drawLineSet[k][idi] == -1 || drawLineSet[k][idj] == -1) {
////						flag = true;
////					}
////				}
////				if (flag == true) {
////					continue;
////				}
////				tuIndex1 = drawLinePicIndex[idi];
////				tuIndex2 = drawLinePicIndex[idj];
////
////				//(2)计算两张图像的基础矩阵
////				//第一种方法，通过两个相机的内外参进行计算
////				View *view1 = my_sfm_data.views.at(tuIndex1).get();
////				openMVG::cameras::IntrinsicBase * cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();
////				openMVG::geometry::Pose3 pose1 = my_sfm_data.GetPoseOrDie(view1);
////
////				View *view2 = my_sfm_data.views.at(tuIndex2).get();
////				openMVG::cameras::IntrinsicBase * cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
////				openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);
////
////				Eigen::Matrix<double, 3, 4> proj1 = cam1->get_projective_equivalent(pose1),
////					proj2 = cam2->get_projective_equivalent(pose2);
////				openMVG::Vec3 c = my_sfm_data.poses[tuIndex1].center();
////
////				cv::Mat promatric1(3, 4, CV_64FC1);
////				cv::Mat promatric2(3, 4, CV_64FC1);
////				for (int i = 0; i < 3; i++) {
////					for (int j = 0; j < 4; j++) {
////						promatric1.at<double>(i, j) = proj1(i, j);
////						promatric2.at<double>(i, j) = proj2(i, j);
////					}
////				}
////				//第一个相机中心
////				cv::Mat C(4, 1, CV_64FC1);
////				for (int i = 0; i < 3; i++) {
////					C.at<double>(i, 0) = c(i, 0);
////				}
////				C.at<double>(3, 0) = 1.0;
////				//计算基础矩阵
////				cv::Mat ee = promatric2*C;
////				cv::Mat eInvSym = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
////				eInvSym.at<double>(0, 1) = -ee.at<double>(2);
////				eInvSym.at<double>(0, 2) = ee.at<double>(1);
////				eInvSym.at<double>(1, 0) = ee.at<double>(2);
////				eInvSym.at<double>(1, 2) = -ee.at<double>(0);
////				eInvSym.at<double>(2, 0) = -ee.at<double>(1);
////				eInvSym.at<double>(2, 1) = ee.at<double>(0);
////				cv::Mat FundamentEPP = eInvSym*promatric2*promatric1.inv(cv::DECOMP_SVD);
////				//cout << idi << idj << endl << FundamentEPP << endl;
////
////				//(3)设置极线图像的路径与图像名称，保留每次计算的极线图像
////				//设置图像名称，注意\\仅在数组中占用一位
////				string epiImg1Path = "epipolarImg/fisrtImg00.jpg";
////				epiImg1Path[20] = tuIndex1 + '0';
////				epiImg1Path[21] = tuIndex2 + '0';
////				epiImg1Path = my_sfm_data.s_root_path + "/../" + epiImg1Path;
////				string epiImg2Path = "epipolarImg/secondImg00.jpg";
////				epiImg2Path[21] = tuIndex1 + '0';
////				epiImg2Path[22] = tuIndex2 + '0';
////				epiImg2Path = my_sfm_data.s_root_path + "/../" + epiImg2Path;
////
////
////				//（4）计算、保存第一张图像的点，并在图像上画出
////				string img1Name = my_sfm_data.s_root_path + "/" + view1->s_Img_path;
////				cv::Mat imageMat1 = cv::imread(img1Name, 1);
////
////				vector<openMVG::Vec2 > points_1, points_2;    //存入点，为后期进行三角测量做准备
////				fstream out(txtPath + "/point.txt", ios::out);
////				//vector<cv::Point2f> point1;
////				for (int k = 0; k < drawLineNum; k++)
////				{
////					cv::line_descriptor::KeyLine keyline = keyLineArray[idi][drawLineSet[k][idi]];
////					double start_x = keyline.startPointX + pointPair[idi].x;
////					double start_y = keyline.startPointY + pointPair[idi].y;
////					double end_x = keyline.endPointX + pointPair[idi].x;
////					double end_y = keyline.endPointY + pointPair[idi].y;
////					//cout << setprecision(15) << "##" << showpoint << start_x << " " << start_y << endl;
////					//cout << setprecision(15) << "##" << showpoint << end_x << " " << end_y << endl;
////					out << setprecision(15) << "##" << showpoint << start_x << " " << start_y << endl;
////					out << setprecision(15) << "##" << showpoint << end_x << " " << end_y << endl;
////					points_1.push_back(openMVG::Vec2(start_x, start_y));
////					points_1.push_back(openMVG::Vec2(end_x, end_y));
////
////					//画图像1的直线和2个端点
////					line(imageMat1, cv::Point2d(start_x, start_y), cv::Point2d(end_x, end_y), cv::Scalar(255, 0, 0), 1);
////					circle(imageMat1, cv::Point2d(start_x, start_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
////					circle(imageMat1, cv::Point2d(end_x, end_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
////				}
////				imwrite(epiImg1Path, imageMat1);
////
////				//(5)计算极线
////				//vector<cv::Point2d> point1;
////				//for (int i = 0; i < points_1.size(); i++) {
////				//	openMVG::Vec2 tmp = cam1->get_ud_pixel(points_1[i]);
////				//	point1.push_back(cv::Point2d(tmp.x(), tmp.y()));
////				//}
////				//cout << "!!!!!!!!!!!!!!!!" << endl;
////				//for (int i = 0; i < point1.size(); i++)
////				//	cout << point1[i] << endl;
////				////采用OPENCV的函数计算极线
////				//vector<cv::Vec3f> corresEpilines;
////				//computeCorrespondEpilines(point1, 1, FundamentEPP, corresEpilines);
////
////				//(5)计算极线
////				vector<cv::Vec3f> corresEpilines;
////				openMVG::Mat3 FF;
////				//把cv：：mat 转化为Eigen
////				for (int i = 0; i < 3; i++)
////				{
////					for (int j = 0; j < 3; j++)
////						FF(i, j) = FundamentEPP.at<double>(i, j);
////				}
////				for (size_t i = 0; i < points_1.size(); i++)
////				{
////					openMVG::Vec2 l_pt = cam1->get_ud_pixel(points_1[i]);
////					openMVG::Vec3 line = FF*openMVG::Vec3(l_pt(0), l_pt(1), 1.0);
////					cv::Vec3f coline(line(0), line(1), line(2));
////					corresEpilines.push_back(coline);
////				}
////
////
////
////				//(6)画图2中的两点对应的两条极线并计算极线与直线的交点
////				string img2Name = my_sfm_data.s_root_path + "/" + view2->s_Img_path;
////				cv::Mat imageMat2 = cv::imread(img2Name, 1);
////				//lineTp = 0;
////				for (size_t k = 0; k < corresEpilines.size(); k++)
////				{
////					float a1 = corresEpilines[k][0];
////					float b1 = corresEpilines[k][1];
////					float c1 = corresEpilines[k][2];
////					// draw the epipolar line between first and last column
////					line(imageMat2, cv::Point(0.0, -c1 / b1), cv::Point(imageMat2.cols, -(c1 + a1 * imageMat2.cols) / b1), cv::Scalar(0, 255, 0), 1.5);
////
////					//另一条线的a,b,c
////					int tp = k / 2;
////					cv::line_descriptor::KeyLine keyline = keyLineArray[idj][drawLineSet[tp][idj]];
////					float a2 = keyline.endPointY - keyline.startPointY;    //y2-y1
////					float b2 = keyline.startPointX - keyline.endPointX;    //x1-x2
////					float c2 = (keyline.endPointX + pointPair[idj].x)*(keyline.startPointY + pointPair[idj].y) - (keyline.startPointX + pointPair[idj].x)*(keyline.endPointY + pointPair[idj].y);    //x2y1-x1y2
////
////																																																	 //画第二张图上的直线
////					if (k % 2 == 0)
////					{
////						//cout << tp << " " << a2 << " " << b2 << " " << c2 << endl;
////
////						line(imageMat2, cv::Point(-c2 / a2, 0.0), cv::Point(-(b2*imageMat2.rows + c2) / a2, imageMat2.rows), cv::Scalar(255, 0, 0), 0.5);
////						//cv::circle(imageMat2, cv::Point(-c2/a2, 0.0), 20, cv::Scalar(0, 0, 255), 10, 8, 0);
////						//cv::circle(imageMat2, cv::Point(-(b2*imageMat2.rows + c2) / a2, imageMat2.rows), 20, cv::Scalar(0, 0, 255), 10, 8, 0);
////					}
////
////
////					//计算交点
////					cv::Point2d ans;
////					ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
////					ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
////					//cout << setprecision(15) << "!!" << showpoint << ans << endl;
////					out << setprecision(15) << ans.x << " " << ans.y << endl;
////					points_2.push_back(openMVG::Vec2(ans.x, ans.y));
////					circle(imageMat2, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
////				}
////				imwrite(epiImg2Path, imageMat2);
////
////
////				//(7)三角测量，并把测量后的点输出
////				openMVG::Triangulation trianObj;
////				std::vector<openMVG::Vec3> points3D;
////
////				for (size_t i = 0; i < points_1.size(); i++) {
////					//cout << "first camera's undistorted pixel coordinate:" << endl << cam1->get_ud_pixel(points_1[i]) << endl;
////					//cout << "second camera's undistorted pixel coordinate:" << endl << cam2->get_ud_pixel(points_2[i]) << endl;
////					trianObj.add(cam1->get_projective_equivalent(pose1), cam1->get_ud_pixel(points_1[i]));
////					trianObj.add(cam2->get_projective_equivalent(pose2), cam2->get_ud_pixel(points_2[i]));
////					//使用旋转转化矩阵，对三角测量后的顶点进行旋转转化，而不是对所有顶点、相机进行转化
////					points3D.push_back(finalrt*trianObj.compute());
////					trianObj.clear();
////				}
////				points3DForGPS = points3D;
////
////				//将测量出的空间点写入文件
////				string points3DPath = my_sfm_data.s_root_path + "/../" + "points3D";
////				mkdir(points3DPath.c_str());
////				string points3Dfile = "points3D/3Dpoints00.txt";
////				points3Dfile[17] = tuIndex1 + '0';
////				points3Dfile[18] = tuIndex2 + '0';
////				fstream outPoints3D(my_sfm_data.s_root_path + "/../" + points3Dfile, ios::out);
////				for (size_t i = 0; i < points3D.size(); i++) {
////					outPoints3D << points3D[i].x() << " " << points3D[i].y() << " " << points3D[i].z() << " " << 255 << " " << 0 << " " << 0 << endl;
////				}
////				outPoints3D.close();
////
////				//(8)计算姿态角
////				string anglePath = my_sfm_data.s_root_path + "/../" + "angle";
////				mkdir(anglePath.c_str());
////				string angle = "angle/angle00.txt";
////				angle[11] = tuIndex1 + '0';
////				angle[12] = tuIndex2 + '0';
////				fstream outAngle(my_sfm_data.s_root_path + "/../" + angle, ios::out);
////				cout << "图像索引对：" << idi << " " << idj << endl;
////				for (unsigned int i = 0; i < points3D.size(); i = i + 2) {
////					double fuyang = getFuYang(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
////					verticalAngles.push_back(fuyang);
////					double shuiping = getShuiPing(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
////					//计算水平矢量，以进行平均水平角和标准差的计算
////					double vecX, vecY;
////					//找到始点与终点，终点为z值更大的那个点
////					if (points3D[i].z() >= points3D[i + 1].z()) {
////						vecX = points3D[i].x() - points3D[i + 1].x();
////						vecY = points3D[i].y() - points3D[i + 1].y();
////					}
////					else {
////						vecX = points3D[i + 1].x() - points3D[i].x();
////						vecY = points3D[i + 1].y() - points3D[i].y();
////					}
////					//添加归一化的向量到容器中
////					horizontalVecs.push_back(cv::Vec2d(vecX / (sqrt(vecX * vecX + vecY * vecY)), vecY / (sqrt(vecX * vecX + vecY * vecY))));
////					cout << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;
////					outAngle << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;
////
////				}
////				cout << endl;
////				outAngle.close();
////			}
////		}
////
////
////		//for (int i = 0; i < drawLineNum; i++)
////		//{
////		//	cout <<"第" <<i << "条直线"<<endl;
////		//	outAvgAngle << "第" << i << "条直线" << endl;
////		//	averFuYang[i] = averFuYang[i] / (ucount*1.0);
////		//	//averShuiPin[i] = averShuiPin[i] / (ucount*1.0);
////		//	averShuiPin[i] = getShuiPing(shuipingVec[0], shuipingVec[1], 1.0, 0.0, 0.0, 0.0);
////		//	cout<< setprecision(15) << "俯仰角：" << averFuYang[i] << endl << "水平角：" << averShuiPin[i] << endl;
////		//	outAvgAngle << setprecision(15) << "俯仰角：" << averFuYang[i] << endl << "水平角：" << averShuiPin[i] << endl;
////		//}
////
////
////		//计算俯仰角平均值与标准差
////		double avgHor = 0.0, avgVer = 0.0, deviationHor = 0.0, deviationVer = 0.0;
////		for (size_t i = 0; i < verticalAngles.size(); i++) {
////			avgVer += verticalAngles[i];
////		}
////		avgVer /= verticalAngles.size();
////
////		for (size_t i = 0; i < verticalAngles.size(); i++) {
////			deviationVer += (verticalAngles[i] - avgVer) * (verticalAngles[i] - avgVer);
////		}
////		deviationVer = sqrt(deviationVer / verticalAngles.size());
////
////		//计算水平角的平均值与标准差
////		cv::Vec2d avgVec(0.0, 0.0);
////		//将归一化的向量相加
////		for (size_t i = 0; i < horizontalVecs.size(); i++) {
////			avgVec[0] += horizontalVecs[i][0];
////			avgVec[1] += horizontalVecs[i][1];
////		}
////		//获取平均水平角
////		avgHor = getShuiPing(avgVec[0], avgVec[1], 1.0, 0.0, 0.0, 0.0);
////		//计算水平角标准差
////		double theta;
////		for (size_t i = 0; i < horizontalVecs.size(); i++) {
////			theta = (180.0 / M_PI) * acos((avgVec[0] * horizontalVecs[i][0] + avgVec[1] * horizontalVecs[i][1]) /
////				(sqrt(avgVec[0] * avgVec[0] + avgVec[1] * avgVec[1])*sqrt(horizontalVecs[i][0] * horizontalVecs[i][0] + horizontalVecs[i][1] * horizontalVecs[i][1])));
////			deviationHor += theta * theta;
////		}
////		deviationHor = sqrt(deviationHor / horizontalVecs.size());
////
////		//输出平均角度至文件
////		string avgAngle = "angle/averageAngle.txt";
////		fstream outAvgAngle(my_sfm_data.s_root_path + "/../" + avgAngle, ios::out);
////		outAvgAngle << "俯仰角与水平角的平均值：" << endl;
////		cout << "俯仰角与水平角的平均值：" << endl;
////		cout << "第0条直线" << endl;
////		outAvgAngle << "第0条直线" << endl;
////		//输出信息
////		cout << setprecision(15) << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
////		cout << setprecision(15) << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;
////		outAvgAngle << setprecision(15) << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
////		outAvgAngle << setprecision(15) << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;
////		outAvgAngle.close();
////
////		//使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
////		for (Landmarks::iterator iterL = my_sfm_data.structure.begin();
////			iterL != my_sfm_data.structure.end(); ++iterL) {
////			iterL->second.X = finalrt*(iterL->second.X);
////		}
////
////		for (Poses::iterator iterP = my_sfm_data.poses.begin();
////			iterP != my_sfm_data.poses.end(); ++iterP) {
////			openMVG::geometry::Pose3 & pose = iterP->second;
////			iterP->second.center() = finalrt*iterP->second.center();
////		}
////
////		//-- Export to disk computed scene (data & visualizable results)
////		std::cout << "...Export SfM_Data to disk." << std::endl;
////
////		Save(my_sfm_data,
////			stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
////			ESfM_Data(ALL));
////
////		Save(my_sfm_data,
////			stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
////			ESfM_Data(ALL));
////
////
////
////		/*******************4、目标GPS与海拔的计算***************/
////		//(1)获取GPS与海拔信息
////		vector<double> tmpGPS;
////		double *longitude = new double[posesNum];
////		double *latitude = new double[posesNum];
////		double *altitude = new double[posesNum];
////		//将有效的poses的横坐标，海拔输入
////		for (size_t i = 0; i < posesNum; i++) {
////			tmpGPS = allGPS[posesIndex[i]];
////			longitude[i] = tmpGPS[0] * DEG_TO_RAD;
////			latitude[i] = tmpGPS[1] * DEG_TO_RAD;
////			altitude[i] = tmpGPS[2];
////		}
////		//获取海拔的众数
////		double commonZ = altitude[0];
////		for (size_t i = 0; i < posesNum; i++) {
////			if (Count(altitude, posesNum, altitude[i]) < Count(altitude, posesNum, altitude[i + 1])) {
////				commonZ = altitude[i + 1];
////			}
////		}
////
////		//(2)将WGS84经纬度转化为墨卡托投影坐标系
////		projPJ pj_merc, pj_latlong;
////		if (!(pj_merc = pj_init_plus("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) {
////			cout << "投影创建失败!" << endl;
////			exit(EXIT_FAILURE);
////		}
////		if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs"))) {
////			cout << "投影创建失败!" << endl;
////			exit(EXIT_FAILURE);
////		}
////		//经纬度至投影坐标系的转换
////		pj_transform(pj_latlong, pj_merc, posesNum, 1, longitude, latitude, NULL);
////		//经纬度数据为0时，投影后存在很小的小数值，在此忽略
////		for (size_t i = 0; i < posesNum; i++) {
////			if (longitude[i] < 1e-5) {
////				longitude[i] = 0.0;
////			}
////			if (latitude[i] < 1e-5) {
////				latitude[i] = 0.0;
////			}
////		}
////		//(3)获取虚拟空间中每个相机的X,Y,Z坐标
////		vector<double> xCoor, yCoor, zCoor;
////		for (Poses::iterator iterP = my_sfm_data.poses.begin();
////			iterP != my_sfm_data.poses.end(); ++iterP)
////		{
////			xCoor.push_back(iterP->second.center().x());
////			yCoor.push_back(iterP->second.center().y());
////			zCoor.push_back(iterP->second.center().z());
////		}
////
////		//(4)使用最小二乘法法计算平移T与缩放S
////		/*
////		Eigen::MatrixXd A(posesNum * 2, 3);
////		Eigen::VectorXd b(posesNum * 2, 1);
////		Eigen::VectorXd x(3, 1);
////
////		for (int i = 0; i < posesNum; i++) {
////		A(i * 2, 1) = 1;
////		A(i * 2, 2) = 0;
////		A(i * 2 + 1, 1) = 0;
////		A(i * 2 + 1, 2) = 1;
////		A(i * 2, 0) = xCoor[i];
////		A(i * 2 + 1, 0) = yCoor[i];
////		b(i * 2) = longitude[i];
////		b(i * 2 + 1) = latitude[i];
////		}
////		cout << A << endl;
////		cout << b << endl;
////		cout << x << endl;
////		x = A.colPivHouseholderQr().solve(b);
////		cout << x << endl;
////		*/
////		//使用先验知识计算平移T，以及缩放S，假设用户拍摄间隔的平均距离为0.5m
////		//计算S
////		Eigen::VectorXd x(3, 1);
////		double gap = 0;
////		for (size_t i = 0; i < posesNum - 1; i++) {
////			gap = gap + sqrt((xCoor[i + 1] - xCoor[i])*(xCoor[i + 1] - xCoor[i]) +
////				(yCoor[i + 1] - yCoor[i])*(yCoor[i + 1] - yCoor[i]));
////		}
////		gap = gap / (posesNum - 1);
////		double S = 0.5 / gap;
////		x(0) = S;
////		//计算T
////		double Tx = 0;
////		double Ty = 0;
////		for (size_t i = 0; i < posesNum; i++) {
////			Tx += longitude[i] - S*xCoor[i];
////			Ty += latitude[i] - S*yCoor[i];
////		}
////		Tx = Tx / posesNum;
////		Ty = Ty / posesNum;
////		x(1) = Tx;
////		x(2) = Ty;
////
////		//(5)对三角测量出来的目标点，进行T,S转化并输出GPS坐标
////		string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
////		mkdir(GPSPath.c_str());
////		string GPSfile = "GPS/GPS.txt";
////		fstream outGPS(my_sfm_data.s_root_path + "/../" + GPSfile, ios::out);
////		double x0 = points3DForGPS[0].x();
////		double y0 = points3DForGPS[0].y();
////		double z0 = points3DForGPS[0].z();
////		x0 = x0 * x(0) + x(1);
////		y0 = y0 * x(0) + x(2);
////		//输出所有平面坐标点
////		outGPS << "投影平面的相机坐标:" << endl;
////		cout << "投影平面的相机坐标:" << endl;
////		for (size_t i = 0; i < posesNum; i++) {
////			cout << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
////			outGPS << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
////		}
////		delete[] longitude;
////		delete[] latitude;
////		delete[] altitude;
////		cout << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;
////		outGPS << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;
////		//将平面坐标逆转化为GPS
////		pj_transform(pj_merc, pj_latlong, 1, 1, &x0, &y0, NULL);
////		cout << setprecision(15) << "S: " << x[0] << endl;
////		cout << setprecision(15) << "Tx: " << x[1] << endl;
////		cout << setprecision(15) << "Ty: " << x[2] << endl << endl;
////		outGPS << setprecision(15) << "S: " << x[0] << endl;
////		outGPS << setprecision(15) << "Tx: " << x[1] << endl;
////		outGPS << setprecision(15) << "Ty: " << x[2] << endl << endl;
////
////		x0 *= RAD_TO_DEG;
////		y0 *= RAD_TO_DEG;
////		//基本在GPS数据为0时才有此情况
////		if (x0 < 1e-3) {
////			x0 = 0.0;
////			y0 = 0.0;
////		}
////		cout << setprecision(15) << "经度：" << x0 << endl;
////		cout << setprecision(15) << "纬度：" << y0 << endl;
////		outGPS << setprecision(15) << "经度：" << x0 << endl;
////		outGPS << setprecision(15) << "纬度：" << y0 << endl;
////		//(6)计算海拔
////		//获取虚拟空间中相机纵坐标的均值
////		double sumOfZ = 0.0;
////		for (size_t i = 0; i < posesNum; i++) {
////			sumOfZ += zCoor[i];
////		}
////		sumOfZ /= posesNum;
////		double Z = commonZ + (z0 - sumOfZ) * x[0];
////		cout << setprecision(15) << "海拔：" << Z << endl;
////		outGPS << setprecision(15) << "海拔：" << Z << endl;
////		outGPS.close();
////		std::system("pause");
////		return EXIT_SUCCESS;
////	}
////	return EXIT_SUCCESS;
////}
////
