#include "main_tx.h"
#include "openMVG\multiview\triangulation_nview.hpp"
/*@param_input sfm_data
*@param_input  posesIndex 有效索引编号
*@param_output rAndroid 从txt文件读取的旋转矩阵
*@param_output allGPS 从txt文件读取的GPS
*@param_input filePath txt存放的根目录
*/
void readRotations(const SfM_Data & sfm_data,
    const vector<int> &posesIndex,
    vector<Eigen::Matrix<double, 3, 3> > &rAndroid,
    vector<vector<double> > &allGPS,
    string &filePath)
{
    vector<Eigen::Matrix<double, 3, 3> > rAndroidAll;
    size_t viewsNum = sfm_data.views.size();
    size_t posesNum = sfm_data.poses.size();

    Eigen::Matrix<double, 3, 3> rotXZ;
    rotXZ << 0, 1, 0, 1, 0, 0, 0, 0, -1;

    for (size_t i = 0; i < viewsNum; ++i)
    {
        Eigen::Matrix<double, 3, 3> tempMat1;
        double tempAngle[3]{ 0.0 };
        double tempGPS[3]{ 0.0 };
        vector<double> tempGPSvec;
        string fileName = filePath + sfm_data.views.at(i)->s_Img_path + ".txt";
        string angleId = "Orientation_NEW_API:";
        getInfoFromTxt(tempAngle, fileName, angleId);
        string GpsId = "GPS:";
        getInfoFromTxt(tempGPS, fileName, GpsId);
        tempGPSvec.push_back(tempGPS[0]);
        tempGPSvec.push_back(tempGPS[1]);
        tempGPSvec.push_back(tempGPS[2]);
        allGPS.push_back(tempGPSvec);
        getRotMatrixZXY(tempMat1, -tempAngle[0], tempAngle[2], -tempAngle[1]);
        rAndroidAll.push_back(tempMat1 * rotXZ);
    }

    for (size_t i = 0; i < posesNum; ++i)
    {
        rAndroid.push_back(rAndroidAll[posesIndex[i]]);
    }
}



/*
*@param_out  out_finalrt	最终合成的旋转角 
*@param_in	poseNum			pose数量 即SfM后最终有效的图片数量
*@param_in rotationsAndroid 安卓得到的旋转角
*@param_in rotations sfm	得到的虚拟空间的旋转
*@param_in fileOutPath		角度结果写入的根目录
*IO  写txt
*/
void virtualToReality(Eigen::Matrix<double, 3, 3> &out_finalrt, int poseNum,
	const vector<Eigen::Matrix<double, 3, 3> > &rotationsAndroid,
	const vector<Eigen::Matrix<double, 3, 3> > &rotations,
	string fileOutPath)
{
	vector<double> eulerVector;
	vector<Eigen::Matrix<double, 3, 3> >rt(poseNum);
	for (size_t i = 0; i < poseNum; ++i)
	{
		double eulerT[3];
		rt[i] = rotationsAndroid[i] * (rotations[i].inverse());
		RotationMatrixToEulerAnglesZYX(rt[i], eulerT);
		eulerVector.push_back(eulerT[0]);
		eulerVector.push_back(eulerT[1]);
		eulerVector.push_back(eulerT[2]);
	}

	int sizeOfEulerVector = poseNum * 3;
	for (int j = 0; j < 3; j++)
	{
		if (abs(eulerVector[j]) > 175)
		{
			size_t positiveNum = 0;
			for (int i = 0; i < sizeOfEulerVector; i += 3)
			{
				positiveNum += (eulerVector[i + j] > 0);
			}
			if (positiveNum < poseNum)
			{
				if (positiveNum < poseNum >> 1)
				{
					for (int i = 0; i < sizeOfEulerVector; i += 3)
					{
						eulerVector[i + j] = abs(eulerVector[i + j]);
					}
				}
				else
				{
					for (int i = 0; i < sizeOfEulerVector; i += 3)
					{
						eulerVector[i + j] = -abs(eulerVector[i + j]);
					}
				}
			}
		}
	}
	double eulerTotal[3] = { 0.0,0.0,0.0 };
	for (int i = 0; i < sizeOfEulerVector; i += 3)
	{
		eulerTotal[0] += eulerVector[i];
		eulerTotal[1] += eulerVector[i + 1];
		eulerTotal[2] += eulerVector[i + 2];
	}

	eulerTotal[0] /= poseNum;
	eulerTotal[1] /= poseNum;
	eulerTotal[2] /= poseNum;

	getRotMatrixZYX(out_finalrt, eulerTotal[0], eulerTotal[1], eulerTotal[2]);



	//计算均方
	double RSME = 0.0;
	for (size_t i = 0; i < eulerVector.size(); i += 3)
	{
		RSME += (abs(eulerTotal[0] - eulerVector[i]) + abs(eulerTotal[1] - eulerVector[i + 1])
			+ abs(eulerTotal[2] - eulerVector[i + 2]));
	}


/********************************************************************************/
	//写入文件
	string txtPath = fileOutPath + "/txtFiles";
	fstream tr0(txtPath + "/transformRot.txt", ios::out);
	for (size_t i = 0; i < poseNum; i++) {
		tr0 << "rt" << i << endl << rt[i] << endl;
	}
	//tr0 << "rt4_2:" << rt4_2 << endl;
	tr0 << "final_rt:" << out_finalrt << endl;
	tr0.close();
/*******************************************************************************/
}


/*
*@param_input smallPicPreName 小图片的共同前缀名称，my_Path/matches/picSmall0
*@param_input picIndex 进行框选的图片编号，与smallPicPreName共同组成小图片的全名, my_Path/mathces/picSmall01.jpg
*@param_output keyLineArray 直线检测结果保存的数组 返回值
*此函数内会进行图片的读取和写入 imread imwrite
*/
void lineDetector(const string &smallPicPreName, const vector<int> &picIndex,
	vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray)
{
	for (auto i: picIndex) {
		string smallPicName = smallPicPreName + to_string(i) + ".jpg";
		string picWithLine = smallPicPreName + to_string(i) + "_with_lines.jpg";
		vector<cv::line_descriptor::KeyLine> lineInOnePic;

		// 读入框的小图片smallPicName， 进行直线检测， 将画上直线的图片写到picWithLine 
		// 返回是直线检测结果
		lineInOnePic = drawLines(smallPicName.c_str(), picWithLine.c_str());
		if (lineInOnePic.empty())
		{
			continue;
		}
		else
		{
			keyLineArray.push_back(lineInOnePic);
		}
	}
}

/*
*@out FundamentEPP
*@in drawLinePicIndex
*@in tuIndex1
*@in tuIndex2
*@in my_sfm_data
*/
void computeFundament(cv::Mat &FundamentEPP, const vector<int> &drawLinePicIndex, int tuIndex1, int tuIndex2,
	 openMVG::sfm::SfM_Data &my_sfm_data)
{
	const View *view1 = my_sfm_data.views.at(tuIndex1).get();
	const openMVG::cameras::IntrinsicBase *cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();
	const openMVG::geometry::Pose3 pose1 = my_sfm_data.GetPoseOrDie(view1);

	const View *view2 = my_sfm_data.views.at(tuIndex2).get();
	const openMVG::cameras::IntrinsicBase *cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
	const openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);

	Eigen::Matrix<double, 3, 4> proj1 = cam1->get_projective_equivalent(pose1);
	Eigen::Matrix<double, 3, 4> proj2 = cam2->get_projective_equivalent(pose2);
	const openMVG::Vec3 c = my_sfm_data.poses[tuIndex1].center();

	cv::Mat promatric1(3, 4, CV_64FC1);
	cv::Mat promatric2(3, 4, CV_64FC1);

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			promatric1.at<double>(i, j) = proj1(i, j);
			promatric2.at<double>(i, j) = proj2(i, j);
		}
	}

	cv::Mat C(4, 1, CV_64FC1);
	for (int i = 0; i < 3; ++i)
	{
		C.at<double>(i, 0) = c(i, 0);
	}
	C.at<double>(3, 0) = 1.0;
	//计算基础矩阵
	cv::Mat ee = promatric2*C;
	cv::Mat eInvSym = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
	eInvSym.at<double>(0, 1) = -ee.at<double>(2);
	eInvSym.at<double>(0, 2) = ee.at<double>(1);
	eInvSym.at<double>(1, 0) = ee.at<double>(2);
	eInvSym.at<double>(1, 2) = -ee.at<double>(0);
	eInvSym.at<double>(2, 0) = -ee.at<double>(1);
	eInvSym.at<double>(2, 1) = ee.at<double>(0);
	FundamentEPP = eInvSym*promatric2*promatric1.inv(cv::DECOMP_SVD);
}

/*
*@out points_1
*@out points_2
*@in my_sfm_data
*@in keyLineArray
*IO 写txt
*/
void computeCorrespondingPoints(vector<openMVG::Vec2> &points_1, vector<openMVG::Vec2> &points_2,
	openMVG::sfm::SfM_Data &my_sfm_data, const vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray,
	const vector<int> &drawLinePair, const vector<cv::Point2d> &pointPair,
	int tuIndex1, int tuIndex2,
	int idi, int idj,
	cv::Mat FundamentEPP)
{
	string epiImg1Path = "epipolarImg/firstImg00.jpg";
	epiImg1Path[20] = (char)tuIndex1 + '0';
	epiImg1Path[21] = (char)tuIndex2 + '0';
	epiImg1Path = my_sfm_data.s_root_path + "/../" + epiImg1Path;
	string epiImg2Path = "epipolarImg/secondImg00.jpg";
	epiImg2Path[21] = (char)tuIndex1 + '0';
	epiImg2Path[22] = (char)tuIndex2 + '0';
	epiImg2Path = my_sfm_data.s_root_path + "/../" + epiImg2Path;

	std::fstream out(my_sfm_data.s_root_path + "/../textfiles/point.txt", std::ios::out);
	
	cv::line_descriptor::KeyLine keyline= keyLineArray[idi][drawLinePair[idi]];
	double start_x = keyline.startPointX + pointPair[idi].x;
	double start_y = keyline.startPointY + pointPair[idi].y;
	double end_x = keyline.endPointX + pointPair[idi].x;
	double end_y = keyline.endPointY + pointPair[idi].y;

	out << std::setprecision(15) << "##" << std::showpoint << start_x << " " << start_y << std::endl;
	out << std::setprecision(15) << "##" << std::showpoint << end_x << " " << end_y << std::endl;

	points_1.push_back(openMVG::Vec2(start_x, start_y));
	points_1.push_back(openMVG::Vec2(end_x, end_y));

	View *view1 = my_sfm_data.views.at(tuIndex1).get();
	View *view2 = my_sfm_data.views.at(tuIndex2).get();

	openMVG::cameras::IntrinsicBase * cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();

	cv::Mat imageMat1 = cv::imread(my_sfm_data.s_root_path + "/" + view1->s_Img_path, 1);
	
	cv::line(imageMat1, cv::Point2d(start_x, start_y), cv::Point2d(end_x, end_y), cv::Scalar(255, 0, 0), 1);
	cv::circle(imageMat1, cv::Point2d(start_x, start_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
	cv::circle(imageMat1, cv::Point2d(end_x, end_y), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
	cv::imwrite(epiImg1Path, imageMat1);

	vector<cv::Vec3f> corresEpilines;
	openMVG::Mat3 FF;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			FF(i, j) = FundamentEPP.at<double>(i, j);
		}
	}

	for (size_t i = 0; i < points_1.size(); ++i)
	{
		openMVG::Vec2 l_pt = cam1->get_ud_pixel(points_1[i]);
		openMVG::Vec3 line = FF*openMVG::Vec3(l_pt(0), l_pt(1), 1.0);
		cv::Vec3f coline(line(0), line(1), line(2));
		corresEpilines.push_back(coline);
	}


	cv::Mat imageMat2 = cv::imread(my_sfm_data.s_root_path + "/" + view2->s_Img_path, 1);
		keyline = keyLineArray[idj][drawLinePair[idj]];
	float a2 = keyline.endPointY - keyline.startPointY;
	float b2 = keyline.startPointX - keyline.endPointX;
	float c2 = (keyline.endPointX + pointPair[idj].x)*(keyline.startPointY + pointPair[idj].y)
		- (keyline.startPointX + pointPair[idj].x)*(keyline.endPointY + pointPair[idj].y);
	cv::line(imageMat2, cv::Point(-c2 / a2, 0.0), cv::Point(-(b2*imageMat2.rows + c2) / a2, imageMat2.rows), cv::Scalar(255, 0, 0), 0.5);
	for (size_t k = 0; k < corresEpilines.size(); ++k)
	{
		float a1 = corresEpilines[k][0];
		float b1 = corresEpilines[k][1];
		float c1 = corresEpilines[k][2];
		// draw the epipolar line between first and last column
		line(imageMat2, cv::Point(0.0, -c1 / b1), cv::Point(imageMat2.cols, -(c1 + a1 * imageMat2.cols) / b1), cv::Scalar(0, 255, 0), 1.5);

		//计算交点
		cv::Point2d ans;
		ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
		ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
		out << setprecision(15) << ans.x << " " << ans.y << endl;
		points_2.push_back(openMVG::Vec2(ans.x, ans.y));
		circle(imageMat2, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
	}
	imwrite(epiImg2Path, imageMat2);
}

//三角测量
void computeTrangulation(const vector<openMVG::Vec2> &points_1, const vector<openMVG::Vec2> &points_2,
	openMVG::sfm::SfM_Data &my_sfm_data, int tuIndex1, int tuIndex2,
	Eigen::Matrix<double, 3, 3> finalrt,
	vector<openMVG::Vec3> &points3DForGPS) 
{

	const View *view1 = my_sfm_data.views.at(tuIndex1).get();
	const openMVG::cameras::IntrinsicBase *cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();
	const openMVG::geometry::Pose3 pose1 = my_sfm_data.GetPoseOrDie(view1);
	const View *view2 = my_sfm_data.views.at(tuIndex2).get();
	const openMVG::cameras::IntrinsicBase *cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
	const openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);

	vector < openMVG::Vec3> point3D;
	openMVG::Triangulation trianObj;
	for (size_t i = 0; i < points_1.size(); ++i)
	{
		trianObj.add(cam1->get_projective_equivalent(pose1), cam1->get_ud_pixel(points_1[i]));
		trianObj.add(cam2->get_projective_equivalent(pose2), cam1->get_ud_pixel(points_2[i]));

		point3D.push_back(finalrt*trianObj.compute());
		trianObj.clear();
	}
	points3DForGPS = std::move(point3D);
}


/*
*IO  写文件txt
*/
void write3DPoints(openMVG::sfm::SfM_Data &my_sfm_data, const vector<openMVG::Vec3> &points3D,
	int tuIndex1, int tuIndex2)
{
	string points3Dfile = "points3D/3Dpoints00.txt";
	points3Dfile[17] = (char)tuIndex1 + '0';
	points3Dfile[18] = (char)tuIndex2 + '0';
	fstream outPoints3D(my_sfm_data.s_root_path + "/../" + points3Dfile, std::ios::out);
	for (size_t i = 0; i < points3D.size(); ++i)
	{
		outPoints3D << points3D[i].x() << ' ' << points3D[i].y() << ' '
			<< points3D[i].z() << ' ' << 255 << ' ' << 0 << ' ' << 0 << std::endl;
	}
	outPoints3D.close();
}

/*
*@out horizontalVecs 
*@out verticalAngles
*IO 文件夹创建 写txt
*/
void computeAngle(openMVG::sfm::SfM_Data &my_sfm_data,
	int tuIndex1, int tuIndex2,
	int idi, int idj,
	vector<openMVG::Vec3> &points3D,
	vector<cv::Vec2d> &horizontalVecs, vector<double> &verticalAngles)
{
	string anglePath = my_sfm_data.s_root_path + "/../" + "angle";
	mkdir(anglePath.c_str());
	string angle = "angle/angle00.txt";
	angle[11] = tuIndex1 + '0';
	angle[12] = tuIndex2 + '0';
	fstream outAngle(my_sfm_data.s_root_path + "/../" + angle, ios::out);
	
	for (unsigned int i = 0; i < points3D.size(); i = i + 2) {
		double fuyang = getFuYang(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
		verticalAngles.push_back(fuyang);
		double shuiping = getShuiPing(points3D[i].x(), points3D[i].y(), points3D[i].z(), points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z());
		//计算水平矢量，以进行平均水平角和标准差的计算
		double vecX, vecY;
		//找到始点与终点，终点为z值更大的那个点
		if (points3D[i].z() >= points3D[i + 1].z()) {
			vecX = points3D[i].x() - points3D[i + 1].x();
			vecY = points3D[i].y() - points3D[i + 1].y();
		}
		else {
			vecX = points3D[i + 1].x() - points3D[i].x();
			vecY = points3D[i + 1].y() - points3D[i].y();
		}
		//添加归一化的向量到容器中
		horizontalVecs.push_back(cv::Vec2d(vecX / (sqrt(vecX * vecX + vecY * vecY)), vecY / (sqrt(vecX * vecX + vecY * vecY))));

		//cout << "图像索引对：" << idi << " " << idj << endl;
		//cout << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;

		outAngle << setprecision(15) << "俯仰角：" << fuyang << endl << "水平角：" << shuiping << endl;

	}
	//cout << endl;
	outAngle.close();
}

/*
*@in  my_sfm_data
*@in  finalrt  
*@in  drawLinePicIndex
*@in  drawLinePair
*@in  pointPair
*@in  keyLineArray
*@out horizontalVecs
*@out vecticalAngles
*@out points3DForGPS
*
*/
void mdzz(openMVG::sfm::SfM_Data &my_sfm_data,
	Eigen::Matrix<double, 3, 3> finalrt,
	const vector<int> &drawLinePicIndex, const vector<int> &drawLinePair,
	const vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray,
	vector<cv::Vec2d> &horizontalVecs, vector<double> &verticalAngles, 
	vector<openMVG::Vec3> &points3DForGPS, const vector<cv::Point2d> &pointPair)
{
	int tuIndex1, tuIndex2;
	int drawLinePicNum = drawLinePicIndex.size();
	for (int idi = 0; idi < drawLinePicNum; ++idi)
	{
		for (int idj = 0; idj < drawLinePicNum; ++idj)
		{
			if (idj == idi)
				continue;
			if (drawLinePair[idi] == -1 || drawLinePair[idj] == -1)
				continue;
			tuIndex1 = drawLinePicIndex[idi];
			tuIndex2 = drawLinePicIndex[idj];

			//const View *view1 = my_sfm_data.views.at(tuIndex1).get();
			//const openMVG::cameras::IntrinsicBase *cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();
			//const openMVG::geometry::Pose3 pose1 = my_sfm_data.GetPoseOrDie(view1);
			//const View *view2 = my_sfm_data.views.at(tuIndex2).get();
			//const openMVG::cameras::IntrinsicBase *cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
			//const openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);

			cv::Mat FundamentEPP;
			computeFundament(FundamentEPP, drawLinePicIndex, tuIndex1, tuIndex2, my_sfm_data);

			vector<openMVG::Vec2> points_1;
			vector<openMVG::Vec2> points_2;
			
			//计算两张图的同名点
			computeCorrespondingPoints(points_1, points_2, my_sfm_data, keyLineArray, 
				drawLinePair, pointPair, tuIndex1, tuIndex2, idi, idj, FundamentEPP);

			//三角测量
			computeTrangulation(points_1, points_2, my_sfm_data, tuIndex1, tuIndex2,
				finalrt, points3DForGPS);

			//三角测量后的点写到 my_sfm_data.s_root_path + /../ + points3D/
			write3DPoints(my_sfm_data, points3DForGPS, tuIndex1, tuIndex2);

			//计算角度
			computeAngle(my_sfm_data, tuIndex1, tuIndex2, idi, idj, points3DForGPS, horizontalVecs, verticalAngles);
		}
	}
}



/*
*算角度平均值
*
*IO 写txt
*/
void computeAVG(const openMVG::sfm::SfM_Data &my_sfm_data,
	const vector<cv::Vec2d> &horizontalVecs, const vector<double> &verticalAngles)
{
	double avgHor = 0.0, avgVer = 0.0, deviationHor = 0.0, deviationVer = 0.0;
	for (size_t i = 0; i < verticalAngles.size(); ++i) {
		avgVer += verticalAngles[i];
	}
	avgVer /= verticalAngles.size();

	for (size_t i = 0; i < verticalAngles.size(); ++i) {
		deviationVer += (verticalAngles[i] - avgVer) * (verticalAngles[i] - avgVer);
	}
	deviationVer = sqrt(deviationVer / verticalAngles.size());


	cv::Vec2d avgVec(0.0, 0.0);
	//将归一化的向量相加
	for (size_t i = 0; i < horizontalVecs.size(); i++) {
		avgVec[0] += horizontalVecs[i][0];
		avgVec[1] += horizontalVecs[i][1];
	}
	//获取平均水平角
	avgHor = getShuiPing(avgVec[0], avgVec[1], 1.0, 0.0, 0.0, 0.0);
	//计算水平角标准差
	double theta;
	for (size_t i = 0; i < horizontalVecs.size(); i++) {
		theta = (180.0 / M_PI) * acos((avgVec[0] * horizontalVecs[i][0] + avgVec[1] * horizontalVecs[i][1]) /
			(sqrt(avgVec[0] * avgVec[0] + avgVec[1] * avgVec[1])*sqrt(horizontalVecs[i][0] * horizontalVecs[i][0] + horizontalVecs[i][1] * horizontalVecs[i][1])));
		deviationHor += theta * theta;
	}
	deviationHor = sqrt(deviationHor / horizontalVecs.size());
	//输出平均角度至文件
	string avgAngle = "angle/averageAngle.txt";
	fstream outAvgAngle(my_sfm_data.s_root_path + "/../" + avgAngle, ios::out);
	//输出信息至文件
	outAvgAngle << setprecision(15) << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
	outAvgAngle << setprecision(15) << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;
	cout << "俯仰角与水平角的平均值：" << endl;
	cout << setprecision(15) << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
	cout << setprecision(15) << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;
	outAvgAngle.close();
}


void GPSandHeight(const vector<vector<double> > &allGPS, const vector<int> &poseIndex,
	openMVG::sfm::SfM_Data &my_sfm_data, const vector<openMVG::Vec3> &points3DForGPS)
{
	vector<double> tmpGPS;
	size_t posesNum = (int)poseIndex.size();
	double *longitude = new double[posesNum];
	double *latitude = new double[posesNum];
	double *altitude = new double[posesNum];

	//获取GPS and 海拔信息.......
	for (size_t i = 0; i < posesNum; ++i)
	{
		tmpGPS = allGPS[poseIndex[i]];
		longitude[i] = tmpGPS[0] * DEG_TO_RAD;
		latitude[i] = tmpGPS[1] * DEG_TO_RAD;
		altitude[i] = tmpGPS[2];
	}

	//海拔众数。。。。。。
	double commonZ = altitude[0];
	for (size_t i = 0; i < posesNum; i++) {
		if (Count(altitude, posesNum, altitude[i]) < Count(altitude, posesNum, altitude[i + 1])) {
			commonZ = altitude[i + 1];
		}
	}

	projPJ pj_merc, pj_latlong;
	if (!(pj_merc = pj_init_plus("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) {
		/*cout << "投影创建失败!" << endl;*/
		exit(EXIT_FAILURE);
	}
	if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs"))) {
		/*cout << "投影创建失败!" << endl;*/
		exit(EXIT_FAILURE);
	}
	//经纬度至投影坐标系的转换
	pj_transform(pj_latlong, pj_merc, posesNum, 1, longitude, latitude, NULL);
	//经纬度数据为0时，投影后存在很小的小数值，在此忽略
	for (size_t i = 0; i < posesNum; i++) {
		if (longitude[i] < 1e-5) {
			longitude[i] = 0.0;
		}
		if (latitude[i] < 1e-5) {
			latitude[i] = 0.0;
		}
	}
	//(3)获取虚拟空间中每个相机的X,Y,Z坐标
	vector<double> xCoor, yCoor, zCoor;
	for (openMVG::sfm::Poses::iterator iterP = my_sfm_data.poses.begin();
		iterP != my_sfm_data.poses.end(); ++iterP) {
		xCoor.push_back(iterP->second.center().x());
		yCoor.push_back(iterP->second.center().y());
		zCoor.push_back(iterP->second.center().z());
	}

	//(4)使用最小二乘法法计算平移T与缩放S
	//使用先验知识计算平移T，以及缩放S，假设用户拍摄间隔的平均距离为0.5m
	//计算S
	Eigen::VectorXd x(3, 1);
	double gap = 0;
	for (size_t i = 0; i < posesNum - 1; i++) {
		gap = gap + sqrt((xCoor[i + 1] - xCoor[i])*(xCoor[i + 1] - xCoor[i]) +
			(yCoor[i + 1] - yCoor[i])*(yCoor[i + 1] - yCoor[i]));
	}
	gap = gap / (posesNum - 1);
	double S = 0.5 / gap;
	x(0) = S;
	//计算T
	double Tx = 0;
	double Ty = 0;
	for (size_t i = 0; i < posesNum; i++) {
		Tx += longitude[i] - S*xCoor[i];
		Ty += latitude[i] - S*yCoor[i];
	}
	Tx = Tx / posesNum;
	Ty = Ty / posesNum;
	x(1) = Tx;
	x(2) = Ty;

	//(5)对三角测量出来的目标点，进行T,S转化并输出GPS坐标
	string GPSfile = "GPS/GPS.txt";
	fstream outGPS(my_sfm_data.s_root_path + "/../" + GPSfile, ios::out);
	double x0 = points3DForGPS[0].x();
	double y0 = points3DForGPS[0].y();
	double z0 = points3DForGPS[0].z();
	x0 = x0 * x(0) + x(1);
	y0 = y0 * x(0) + x(2);
	//输出所有平面坐标点
	outGPS << "投影平面的相机坐标:" << endl;
	/*cout << "投影平面的相机坐标:" << endl;*/
	for (size_t i = 0; i < posesNum; i++) {
		/*cout << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;*/
		outGPS << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
	}
	delete[] longitude;
	delete[] latitude;
	delete[] altitude;
	/*cout << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;*/
	outGPS << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;
	//将平面坐标逆转化为GPS
	pj_transform(pj_merc, pj_latlong, 1, 1, &x0, &y0, NULL);

	//cout << setprecision(15) << "S: " << x[0] << endl;
	//cout << setprecision(15) << "Tx: " << x[1] << endl;
	//cout << setprecision(15) << "Ty: " << x[2] << endl << endl;
	outGPS << setprecision(15) << "S: " << x[0] << endl;
	outGPS << setprecision(15) << "Tx: " << x[1] << endl;
	outGPS << setprecision(15) << "Ty: " << x[2] << endl << endl;
	x0 *= RAD_TO_DEG;
	y0 *= RAD_TO_DEG;
	//基本在GPS数据为0时才有此情况
	if (x0 < 1e-3) {
		x0 = 0.0;
		y0 = 0.0;
	}
	outGPS << setprecision(15) << "经度：" << x0 << endl;
	outGPS << setprecision(15) << "纬度：" << y0 << endl;
	//(6)计算海拔
	//获取虚拟空间中相机纵坐标的均值
	double sumOfZ = 0.0;
	for (size_t i = 0; i < posesNum; i++) {
		sumOfZ += zCoor[i];
	}
	sumOfZ /= posesNum;
	double Z = commonZ + (z0 - sumOfZ) * x[0];
	/*cout << setprecision(15) << "海拔：" << Z << endl;*/
	outGPS << setprecision(15) << "海拔：" << Z << endl;
	outGPS.close();
}


