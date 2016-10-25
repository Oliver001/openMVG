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
//	/*********************1����תת������ļ���********************/
//	//(1)��ȡSFM DATA
//	SfM_Data my_sfm_data;
//	if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
//		std::cerr << std::endl
//			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
//		return EXIT_FAILURE;
//	}
//
//
//
//	//��Ҫ�������ĸ��ļ���
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
//	//(2)��ȡ����ռ����ת����
//	size_t viewsNum = my_sfm_data.views.size();
//	size_t posesNum = my_sfm_data.poses.size();
//	vector<Eigen::Matrix<double, 3, 3>> rotations;
//	vector<int> posesIndex;
//	for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
//		rotations.push_back(itr->second.rotation());
//		//�洢��Чposes��������
//		posesIndex.push_back(itr->first);
//	}
//
//	//(3)��ȡ��ʵ�ռ��е���ת����
//	//(4)���ֻ�����ת��Ϊ�������Remap
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
//	// (1)�����Ч�ġ����Խ������ǲ�����������,��
//	// �û�ѡ�񣬲�������Ҫ��������������ÿ�ŵ�������
//
//	int drawLinePicNum; ////�û��������ȡ��ͼ�������
//	vector<int> drawLinePicIndex;  //����ֱ����ȡ��ͼƬ���
//
//	std::cout << "��Ч��������:";
//	for (size_t i = 0; i < posesIndex.size(); i++)
//		cout << posesIndex[i] << " ";
//	std::cout << endl;
//	std::cout << "����Ҫƥ���������";
//
//	std::cin >> drawLinePicNum;
//	std::cout << "����ÿ�ŵ�������(ÿ�ŵ������ţ�����һ��)��";
//	for (int i = 0; i < drawLinePicNum; i++) {
//		int tmp;
//		std::cin >> tmp;
//		drawLinePicIndex.push_back(tmp);
//	}
//
//	//(2)����ÿһ��ͼ��
//	//��ͼ 
//	vector<cv::Point2d> pointPair; // ÿ��Сͼ���Ͻǵ����� 
//	for (auto i : drawLinePicIndex)
//	{
//		View *view = my_sfm_data.views.at(i).get();
//		string imgName = my_sfm_data.s_root_path + "/" + view->s_Img_path;
//		image_path = my_sfm_data.s_root_path + "/../" + "matches/picSmall0" + to_string(i) + ".jpg";
//		capture(imgName); //�Ӵ�ͼƬ������ߣ���СͼƬд�� image_path ͬʱ��СͼƬ�����Ͻ�����д��image_path.txt
//						  //��matchesĿ¼��
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
//  //  �����ѡ����ͼƬ�Ĺ�ͬǰ׺�����˴���.../matches/picSmall0, ���ݿ�ѡ�ĵ�ͼƬ��źϳ�ͼƬ��
//  //  ��.../matches/picSmall01.jpg
//  //  ֱ�߼����������keyLineArray
//  //  ͬʱ������ֱ�ߵ�ͼƬд��ͬһ��Ŀ¼�� ��.../matches/picSmalee01_with_lines.jpg
//
//	vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//����ֱ����ȡ���
//	lineDetector(my_sfm_data.s_root_path + "/../" + "matches/picSmall0", drawLinePicIndex, keyLineArray); // ֱ����ȡ
//
//
//	/*******************                   ANDROID                       *****************/
//	//��ȡ����ֱ�ߵ�СͼƬ ��ʾ��
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
//	//  (3)�û�����ֱ�߶Ե��������������Ӧ�ı�ţ���-1����ʾ�˴���ֱ��û�б���ȡ������
//
//	std::vector<int> drawLinePair;// ����ÿ��СͼƬ��ѡ����ֱ�߱��
//	std::cout << "����ֱ����ÿ��ͼ���ϵı��, ��ĳ��ͼ���ϴ�ֱ��û����ȡ�����������롮-1��:" << endl;
//
//	int validNumOfImgs = drawLinePicNum;//���н�����ֱ����ȡ��ͼƬ������
//
//	for (int j = 0; j < drawLinePicNum; j++) {
//		int tmp;
//		std::cin >> tmp;
//		// ������-1������Чͼ������1
//		if (tmp == -1) {
//			validNumOfImgs--;
//		}
//		drawLinePair.push_back(tmp);
//	}
//	cv::destroyAllWindows();
//
///**************************************************************************************/
//  //Ϊ����GPS�洢��Ŀ��������ռ��еĵ�
//  std::vector<openMVG::Vec3> points3DForGPS;
// // ����ÿ�μ��������ˮƽ���������Լ���ƽ��ˮƽ�Ǻ�ˮƽ�ǵı�׼��
//  std::vector<cv::Vec2d> horizontalVecs;
//  std::vector<double> verticalAngles;
// 
//  mdzz(my_sfm_data, finalrt, drawLinePicIndex, drawLinePair,
//	  keyLineArray, horizontalVecs, verticalAngles,
//	  points3DForGPS, pointPair);                           //�Ƕȼ��� ��� horizontalVecs verticalAngles points3DForGPS
//
//  computeAVG(my_sfm_data, horizontalVecs, verticalAngles);
//
//  /*******************4��Ŀ��GPS�뺣�εļ���***************/
//
//  GPSandHeight(allGPS, posesIndex, my_sfm_data, points3DForGPS);
//  
//  
//  //ʹ����תת�����󣬶������������ƽ�����ת����ΪҪ���������ת��ĵ�����̬����������������У���һ���ǿ�ѡ��
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
////		/*********************1����תת������ļ���********************/
////		//(1)��ȡSFM DATA
////		SfM_Data my_sfm_data = sfmEngine.Get_SfM_Data();
////
////		//(2)��ȡ����ռ����ת����
////		size_t viewsNum = my_sfm_data.views.size();
////		size_t posesNum = my_sfm_data.poses.size();
////		vector<Eigen::Matrix<double, 3, 3>> rotations;
////		vector<int> posesIndex;
////		for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
////			rotations.push_back(itr->second.rotation());
////			//�洢��Чposes��������
////			posesIndex.push_back(itr->first);
////		}
////
////		//(3)��ȡ��ʵ�ռ��е���ת����
////		//(4)���ֻ�����ת��Ϊ�������Remap
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
////		//(5)������תת������XYZ, ZXY, ZYX���ַֽⷽʽ�����㣬���ѡȡ��ZYX�ķ�ʽ
////		cout << "������ռ���ת����ʵ�ռ����ת����:" << endl;
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
////		//(6)����ת����ֽ����ת��ʱ�����ֽܷ��+179��-179������������ֵ����ϴ�
////		//���ǽǶ����Ǻܽӽ���������Ҫ�ж�����һ��������Ϊͬһ������
////		//�ж��Ƿ����180���ҵ���ֵ,��һ����ֵ���ڴ���175����Ҫ���
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
////		//(7)����תת������ֽ����������ת����ƽ��ֵ�����ع������ŵ���ת����
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
////		//(8)�������ֵ
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
////		//(9)����TXTĿ¼���洢��Ϣ
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
////		/*********2��ͨ��ֱ����ȡ�뼫��Լ����ȡ������Խ������ǲ���****************/
////
////
////		//(1)�����Ч�ġ����Խ������ǲ�����������,���û�ѡ�񣬲�������Ҫ��������������ÿ�ŵ�������
////		std::cout << "��Ч��������:";
////		for (size_t i = 0; i < posesIndex.size(); i++)
////			cout << posesIndex[i] << " ";
////		std::cout << endl;
////		int drawLinePicNum;                   //�û��������ȡ��ͼ�������
////		vector<int> drawLinePicIndex;         //Ҫ������ȡͼ���������
////		vector<cv::Point2d> pointPair;
////		std::cout << "����Ҫƥ���������";
////		std::cin >> drawLinePicNum;
////		std::cout << "����ÿ�ŵ�������(ÿ�ŵ������ţ�����һ��)��";
////		for (int i = 0; i<drawLinePicNum; i++)
////		{
////			int tmp;
////			std::cin >> tmp;
////			drawLinePicIndex.push_back(tmp);
////		}
////
////
////		//(2)����ÿһ��ͼ��
////		vector<vector<cv::line_descriptor::KeyLine>> keyLineArray;
////		for (int i = 0; i<drawLinePicNum; i++)
////		{
////			//��ȡ����ռ���ÿһ��ͼ���view��������ͼ���ڲΣ����
////			View *view = my_sfm_data.views.at(drawLinePicIndex[i]).get();
////			//��ȡ�����
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
////			//���ÿ��ͼ�ν���ֱ����ȡ���������ֱ�߱�����drawLineKL
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
////			//��ʾͼ��
////			cv::Mat imgMat = cv::imread(image_out_path, 1);
////			if (imgMat.data == NULL) {
////				cout << "Error, images could not be loaded. Please, check their path" << endl;
////				return EXIT_FAILURE;
////			}
////			string windowName = to_string(i) + "ͼƬ";
////			imshow(windowName, imgMat);
////
////#endif // !_WIN32
////		}
////		cvWaitKey(0);
////
////
////		//(3)�û�����ֱ�߶Ե��������������Ӧ�ı�ţ���-1����ʾ�˴���ֱ��û�б���ȡ������
////#ifdef _WIN32
////		int drawLineNum = 1;
////		std::vector<std::vector<int>> drawLineSet;
////		//std::cout << "�����������ֱ�ߵĸ�����";
////		//std::cin >> drawLineNum;
////
////		std::cout << "����ֱ����ÿ��ͼ���ϵı��, ��ĳ��ͼ���ϴ�ֱ��û����ȡ�����������롮-1��:" << endl;
////		//���ĳͼ���ϴ�ֱ��û����ȡ�����������롮-1��
////		//�ܹ���ȡ��ֱ�ߵ�ͼ������
////		int validNumOfImgs = drawLinePicNum;
////		for (int i = 0; i < drawLineNum; i++)
////		{
////			std::vector<int> drawLinePair;
////			for (int j = 0; j < drawLinePicNum; j++)
////			{
////				int tmp;
////				std::cin >> tmp;
////				//������-1������Чͼ������1
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
////		/**************3����������ͼ��Ķ���ԣ�Ȼ��������ǲ�������������̬��**************/
////		//�˹��̿���һ����ѡ�����ͼ�񣬲���������ͼ��֮��Ľ��������������õ���ƽ��ֵ
////		//�����洢���ߵ�·��
////		mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());
////
////		//�����Ǻ�ˮƽ�ǵ�ƽ��ֵ
////		//vector<double> averShuiPin(drawLineNum);
////		//vector<double> averFuYang(drawLineNum);
////		//for (int i = 0; i < drawLineNum; i++)
////		//{
////		//	averFuYang[i] = 0.0;
////		//	averShuiPin[i] = 0.0;
////		//}
////		//Ϊ����GPS�洢��Ŀ��������ռ��еĵ�
////		std::vector<openMVG::Vec3> points3DForGPS;
////		int tuIndex1, tuIndex2;      //����ͼ���������
////									 //����ÿ�μ��������ˮƽ���������Լ���ƽ��ˮƽ�Ǻ�ˮƽ�ǵı�׼��
////		std::vector<cv::Vec2d> horizontalVecs;
////		std::vector<double> verticalAngles;
////		for (int idi = 0; idi < drawLinePicNum; idi++)
////		{
////			for (int idj = 0; idj < drawLinePicNum; idj++)
////			{
////				//(1)���ͼ��������ţ�Ҫ����һ��ͼ�񣬲�������
////				if (idi == idj)
////					continue;
////				bool flag = false;
////				//����-1����������ͼ����㣬�˴��Բ�ֱͬ��û������Դ�
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
////				//(2)��������ͼ��Ļ�������
////				//��һ�ַ�����ͨ���������������ν��м���
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
////				//��һ���������
////				cv::Mat C(4, 1, CV_64FC1);
////				for (int i = 0; i < 3; i++) {
////					C.at<double>(i, 0) = c(i, 0);
////				}
////				C.at<double>(3, 0) = 1.0;
////				//�����������
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
////				//(3)���ü���ͼ���·����ͼ�����ƣ�����ÿ�μ���ļ���ͼ��
////				//����ͼ�����ƣ�ע��\\����������ռ��һλ
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
////				//��4�����㡢�����һ��ͼ��ĵ㣬����ͼ���ϻ���
////				string img1Name = my_sfm_data.s_root_path + "/" + view1->s_Img_path;
////				cv::Mat imageMat1 = cv::imread(img1Name, 1);
////
////				vector<openMVG::Vec2 > points_1, points_2;    //����㣬Ϊ���ڽ������ǲ�����׼��
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
////					//��ͼ��1��ֱ�ߺ�2���˵�
////					line(imageMat1, cv::Point2d(start_x, start_y), cv::Point2d(end_x, end_y), cv::Scalar(255, 0, 0), 1);
////					circle(imageMat1, cv::Point2d(start_x, start_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
////					circle(imageMat1, cv::Point2d(end_x, end_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
////				}
////				imwrite(epiImg1Path, imageMat1);
////
////				//(5)���㼫��
////				//vector<cv::Point2d> point1;
////				//for (int i = 0; i < points_1.size(); i++) {
////				//	openMVG::Vec2 tmp = cam1->get_ud_pixel(points_1[i]);
////				//	point1.push_back(cv::Point2d(tmp.x(), tmp.y()));
////				//}
////				//cout << "!!!!!!!!!!!!!!!!" << endl;
////				//for (int i = 0; i < point1.size(); i++)
////				//	cout << point1[i] << endl;
////				////����OPENCV�ĺ������㼫��
////				//vector<cv::Vec3f> corresEpilines;
////				//computeCorrespondEpilines(point1, 1, FundamentEPP, corresEpilines);
////
////				//(5)���㼫��
////				vector<cv::Vec3f> corresEpilines;
////				openMVG::Mat3 FF;
////				//��cv����mat ת��ΪEigen
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
////				//(6)��ͼ2�е������Ӧ���������߲����㼫����ֱ�ߵĽ���
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
////					//��һ���ߵ�a,b,c
////					int tp = k / 2;
////					cv::line_descriptor::KeyLine keyline = keyLineArray[idj][drawLineSet[tp][idj]];
////					float a2 = keyline.endPointY - keyline.startPointY;    //y2-y1
////					float b2 = keyline.startPointX - keyline.endPointX;    //x1-x2
////					float c2 = (keyline.endPointX + pointPair[idj].x)*(keyline.startPointY + pointPair[idj].y) - (keyline.startPointX + pointPair[idj].x)*(keyline.endPointY + pointPair[idj].y);    //x2y1-x1y2
////
////																																																	 //���ڶ���ͼ�ϵ�ֱ��
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
////					//���㽻��
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
////				//(7)���ǲ��������Ѳ�����ĵ����
////				openMVG::Triangulation trianObj;
////				std::vector<openMVG::Vec3> points3D;
////
////				for (size_t i = 0; i < points_1.size(); i++) {
////					//cout << "first camera's undistorted pixel coordinate:" << endl << cam1->get_ud_pixel(points_1[i]) << endl;
////					//cout << "second camera's undistorted pixel coordinate:" << endl << cam2->get_ud_pixel(points_2[i]) << endl;
////					trianObj.add(cam1->get_projective_equivalent(pose1), cam1->get_ud_pixel(points_1[i]));
////					trianObj.add(cam2->get_projective_equivalent(pose2), cam2->get_ud_pixel(points_2[i]));
////					//ʹ����תת�����󣬶����ǲ�����Ķ��������תת���������Ƕ����ж��㡢�������ת��
////					points3D.push_back(finalrt*trianObj.compute());
////					trianObj.clear();
////				}
////				points3DForGPS = points3D;
////
////				//���������Ŀռ��д���ļ�
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
////				//(8)������̬��
////				string anglePath = my_sfm_data.s_root_path + "/../" + "angle";
////				mkdir(anglePath.c_str());
////				string angle = "angle/angle00.txt";
////				angle[11] = tuIndex1 + '0';
////				angle[12] = tuIndex2 + '0';
////				fstream outAngle(my_sfm_data.s_root_path + "/../" + angle, ios::out);
////				cout << "ͼ�������ԣ�" << idi << " " << idj << endl;
////				for (unsigned int i = 0; i < points3D.size(); i = i + 2) {
////					double fuyang = getFuYang(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
////					verticalAngles.push_back(fuyang);
////					double shuiping = getShuiPing(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
////					//����ˮƽʸ�����Խ���ƽ��ˮƽ�Ǻͱ�׼��ļ���
////					double vecX, vecY;
////					//�ҵ�ʼ�����յ㣬�յ�Ϊzֵ������Ǹ���
////					if (points3D[i].z() >= points3D[i + 1].z()) {
////						vecX = points3D[i].x() - points3D[i + 1].x();
////						vecY = points3D[i].y() - points3D[i + 1].y();
////					}
////					else {
////						vecX = points3D[i + 1].x() - points3D[i].x();
////						vecY = points3D[i + 1].y() - points3D[i].y();
////					}
////					//��ӹ�һ����������������
////					horizontalVecs.push_back(cv::Vec2d(vecX / (sqrt(vecX * vecX + vecY * vecY)), vecY / (sqrt(vecX * vecX + vecY * vecY))));
////					cout << setprecision(15) << "�����ǣ�" << fuyang << endl << "ˮƽ�ǣ�" << shuiping << endl;
////					outAngle << setprecision(15) << "�����ǣ�" << fuyang << endl << "ˮƽ�ǣ�" << shuiping << endl;
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
////		//	cout <<"��" <<i << "��ֱ��"<<endl;
////		//	outAvgAngle << "��" << i << "��ֱ��" << endl;
////		//	averFuYang[i] = averFuYang[i] / (ucount*1.0);
////		//	//averShuiPin[i] = averShuiPin[i] / (ucount*1.0);
////		//	averShuiPin[i] = getShuiPing(shuipingVec[0], shuipingVec[1], 1.0, 0.0, 0.0, 0.0);
////		//	cout<< setprecision(15) << "�����ǣ�" << averFuYang[i] << endl << "ˮƽ�ǣ�" << averShuiPin[i] << endl;
////		//	outAvgAngle << setprecision(15) << "�����ǣ�" << averFuYang[i] << endl << "ˮƽ�ǣ�" << averShuiPin[i] << endl;
////		//}
////
////
////		//���㸩����ƽ��ֵ���׼��
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
////		//����ˮƽ�ǵ�ƽ��ֵ���׼��
////		cv::Vec2d avgVec(0.0, 0.0);
////		//����һ�����������
////		for (size_t i = 0; i < horizontalVecs.size(); i++) {
////			avgVec[0] += horizontalVecs[i][0];
////			avgVec[1] += horizontalVecs[i][1];
////		}
////		//��ȡƽ��ˮƽ��
////		avgHor = getShuiPing(avgVec[0], avgVec[1], 1.0, 0.0, 0.0, 0.0);
////		//����ˮƽ�Ǳ�׼��
////		double theta;
////		for (size_t i = 0; i < horizontalVecs.size(); i++) {
////			theta = (180.0 / M_PI) * acos((avgVec[0] * horizontalVecs[i][0] + avgVec[1] * horizontalVecs[i][1]) /
////				(sqrt(avgVec[0] * avgVec[0] + avgVec[1] * avgVec[1])*sqrt(horizontalVecs[i][0] * horizontalVecs[i][0] + horizontalVecs[i][1] * horizontalVecs[i][1])));
////			deviationHor += theta * theta;
////		}
////		deviationHor = sqrt(deviationHor / horizontalVecs.size());
////
////		//���ƽ���Ƕ����ļ�
////		string avgAngle = "angle/averageAngle.txt";
////		fstream outAvgAngle(my_sfm_data.s_root_path + "/../" + avgAngle, ios::out);
////		outAvgAngle << "��������ˮƽ�ǵ�ƽ��ֵ��" << endl;
////		cout << "��������ˮƽ�ǵ�ƽ��ֵ��" << endl;
////		cout << "��0��ֱ��" << endl;
////		outAvgAngle << "��0��ֱ��" << endl;
////		//�����Ϣ
////		cout << setprecision(15) << "�����ǣ�" << avgVer << endl << "��׼�" << deviationVer << endl;
////		cout << setprecision(15) << "ˮƽ�ǣ�" << avgHor << endl << "��׼�" << deviationHor << endl;
////		outAvgAngle << setprecision(15) << "�����ǣ�" << avgVer << endl << "��׼�" << deviationVer << endl;
////		outAvgAngle << setprecision(15) << "ˮƽ�ǣ�" << avgHor << endl << "��׼�" << deviationHor << endl;
////		outAvgAngle.close();
////
////		//ʹ����תת�����󣬶������������ƽ�����ת����ΪҪ���������ת��ĵ�����̬����������������У���һ���ǿ�ѡ��
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
////		/*******************4��Ŀ��GPS�뺣�εļ���***************/
////		//(1)��ȡGPS�뺣����Ϣ
////		vector<double> tmpGPS;
////		double *longitude = new double[posesNum];
////		double *latitude = new double[posesNum];
////		double *altitude = new double[posesNum];
////		//����Ч��poses�ĺ����꣬��������
////		for (size_t i = 0; i < posesNum; i++) {
////			tmpGPS = allGPS[posesIndex[i]];
////			longitude[i] = tmpGPS[0] * DEG_TO_RAD;
////			latitude[i] = tmpGPS[1] * DEG_TO_RAD;
////			altitude[i] = tmpGPS[2];
////		}
////		//��ȡ���ε�����
////		double commonZ = altitude[0];
////		for (size_t i = 0; i < posesNum; i++) {
////			if (Count(altitude, posesNum, altitude[i]) < Count(altitude, posesNum, altitude[i + 1])) {
////				commonZ = altitude[i + 1];
////			}
////		}
////
////		//(2)��WGS84��γ��ת��Ϊī����ͶӰ����ϵ
////		projPJ pj_merc, pj_latlong;
////		if (!(pj_merc = pj_init_plus("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) {
////			cout << "ͶӰ����ʧ��!" << endl;
////			exit(EXIT_FAILURE);
////		}
////		if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs"))) {
////			cout << "ͶӰ����ʧ��!" << endl;
////			exit(EXIT_FAILURE);
////		}
////		//��γ����ͶӰ����ϵ��ת��
////		pj_transform(pj_latlong, pj_merc, posesNum, 1, longitude, latitude, NULL);
////		//��γ������Ϊ0ʱ��ͶӰ����ں�С��С��ֵ���ڴ˺���
////		for (size_t i = 0; i < posesNum; i++) {
////			if (longitude[i] < 1e-5) {
////				longitude[i] = 0.0;
////			}
////			if (latitude[i] < 1e-5) {
////				latitude[i] = 0.0;
////			}
////		}
////		//(3)��ȡ����ռ���ÿ�������X,Y,Z����
////		vector<double> xCoor, yCoor, zCoor;
////		for (Poses::iterator iterP = my_sfm_data.poses.begin();
////			iterP != my_sfm_data.poses.end(); ++iterP)
////		{
////			xCoor.push_back(iterP->second.center().x());
////			yCoor.push_back(iterP->second.center().y());
////			zCoor.push_back(iterP->second.center().z());
////		}
////
////		//(4)ʹ����С���˷�������ƽ��T������S
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
////		//ʹ������֪ʶ����ƽ��T���Լ�����S�������û���������ƽ������Ϊ0.5m
////		//����S
////		Eigen::VectorXd x(3, 1);
////		double gap = 0;
////		for (size_t i = 0; i < posesNum - 1; i++) {
////			gap = gap + sqrt((xCoor[i + 1] - xCoor[i])*(xCoor[i + 1] - xCoor[i]) +
////				(yCoor[i + 1] - yCoor[i])*(yCoor[i + 1] - yCoor[i]));
////		}
////		gap = gap / (posesNum - 1);
////		double S = 0.5 / gap;
////		x(0) = S;
////		//����T
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
////		//(5)�����ǲ���������Ŀ��㣬����T,Sת�������GPS����
////		string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
////		mkdir(GPSPath.c_str());
////		string GPSfile = "GPS/GPS.txt";
////		fstream outGPS(my_sfm_data.s_root_path + "/../" + GPSfile, ios::out);
////		double x0 = points3DForGPS[0].x();
////		double y0 = points3DForGPS[0].y();
////		double z0 = points3DForGPS[0].z();
////		x0 = x0 * x(0) + x(1);
////		y0 = y0 * x(0) + x(2);
////		//�������ƽ�������
////		outGPS << "ͶӰƽ����������:" << endl;
////		cout << "ͶӰƽ����������:" << endl;
////		for (size_t i = 0; i < posesNum; i++) {
////			cout << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
////			outGPS << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
////		}
////		delete[] longitude;
////		delete[] latitude;
////		delete[] altitude;
////		cout << "Ŀ������" << ": " << x0 << " " << y0 << endl << endl;
////		outGPS << "Ŀ������" << ": " << x0 << " " << y0 << endl << endl;
////		//��ƽ��������ת��ΪGPS
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
////		//������GPS����Ϊ0ʱ���д����
////		if (x0 < 1e-3) {
////			x0 = 0.0;
////			y0 = 0.0;
////		}
////		cout << setprecision(15) << "���ȣ�" << x0 << endl;
////		cout << setprecision(15) << "γ�ȣ�" << y0 << endl;
////		outGPS << setprecision(15) << "���ȣ�" << x0 << endl;
////		outGPS << setprecision(15) << "γ�ȣ�" << y0 << endl;
////		//(6)���㺣��
////		//��ȡ����ռ������������ľ�ֵ
////		double sumOfZ = 0.0;
////		for (size_t i = 0; i < posesNum; i++) {
////			sumOfZ += zCoor[i];
////		}
////		sumOfZ /= posesNum;
////		double Z = commonZ + (z0 - sumOfZ) * x[0];
////		cout << setprecision(15) << "���Σ�" << Z << endl;
////		outGPS << setprecision(15) << "���Σ�" << Z << endl;
////		outGPS.close();
////		std::system("pause");
////		return EXIT_SUCCESS;
////	}
////	return EXIT_SUCCESS;
////}
////
