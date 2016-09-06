#include "main_tx.h"
#include "openMVG\multiview\triangulation_nview.hpp"
/*@param_input sfm_data
*@param_input  posesIndex ��Ч�������
*@param_output rAndroid ��txt�ļ���ȡ����ת����
*@param_output allGPS ��txt�ļ���ȡ��GPS
*@param_input filePath txt��ŵĸ�Ŀ¼
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
*@param_out  out_finalrt	���պϳɵ���ת�� 
*@param_in	poseNum			pose���� ��SfM��������Ч��ͼƬ����
*@param_in rotationsAndroid ��׿�õ�����ת��
*@param_in rotations sfm	�õ�������ռ����ת
*@param_in fileOutPath		�ǶȽ��д��ĸ�Ŀ¼
*IO  дtxt
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



	//�������
	double RSME = 0.0;
	for (size_t i = 0; i < eulerVector.size(); i += 3)
	{
		RSME += (abs(eulerTotal[0] - eulerVector[i]) + abs(eulerTotal[1] - eulerVector[i + 1])
			+ abs(eulerTotal[2] - eulerVector[i + 2]));
	}


/********************************************************************************/
	//д���ļ�
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
*@param_input smallPicPreName СͼƬ�Ĺ�ͬǰ׺���ƣ�my_Path/matches/picSmall0
*@param_input picIndex ���п�ѡ��ͼƬ��ţ���smallPicPreName��ͬ���СͼƬ��ȫ��, my_Path/mathces/picSmall01.jpg
*@param_output keyLineArray ֱ�߼������������� ����ֵ
*�˺����ڻ����ͼƬ�Ķ�ȡ��д�� imread imwrite
*/
void lineDetector(const string &smallPicPreName, const vector<int> &picIndex,
	vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray)
{
	for (auto i: picIndex) {
		string smallPicName = smallPicPreName + to_string(i) + ".jpg";
		string picWithLine = smallPicPreName + to_string(i) + "_with_lines.jpg";
		vector<cv::line_descriptor::KeyLine> lineInOnePic;

		// ������СͼƬsmallPicName�� ����ֱ�߼�⣬ ������ֱ�ߵ�ͼƬд��picWithLine 
		// ������ֱ�߼����
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
	//�����������
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
*IO дtxt
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

		//���㽻��
		cv::Point2d ans;
		ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
		ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
		out << setprecision(15) << ans.x << " " << ans.y << endl;
		points_2.push_back(openMVG::Vec2(ans.x, ans.y));
		circle(imageMat2, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
	}
	imwrite(epiImg2Path, imageMat2);
}

//���ǲ���
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
*IO  д�ļ�txt
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
*IO �ļ��д��� дtxt
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
		//����ˮƽʸ�����Խ���ƽ��ˮƽ�Ǻͱ�׼��ļ���
		double vecX, vecY;
		//�ҵ�ʼ�����յ㣬�յ�Ϊzֵ������Ǹ���
		if (points3D[i].z() >= points3D[i + 1].z()) {
			vecX = points3D[i].x() - points3D[i + 1].x();
			vecY = points3D[i].y() - points3D[i + 1].y();
		}
		else {
			vecX = points3D[i + 1].x() - points3D[i].x();
			vecY = points3D[i + 1].y() - points3D[i].y();
		}
		//��ӹ�һ����������������
		horizontalVecs.push_back(cv::Vec2d(vecX / (sqrt(vecX * vecX + vecY * vecY)), vecY / (sqrt(vecX * vecX + vecY * vecY))));

		//cout << "ͼ�������ԣ�" << idi << " " << idj << endl;
		//cout << setprecision(15) << "�����ǣ�" << fuyang << endl << "ˮƽ�ǣ�" << shuiping << endl;

		outAngle << setprecision(15) << "�����ǣ�" << fuyang << endl << "ˮƽ�ǣ�" << shuiping << endl;

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
			
			//��������ͼ��ͬ����
			computeCorrespondingPoints(points_1, points_2, my_sfm_data, keyLineArray, 
				drawLinePair, pointPair, tuIndex1, tuIndex2, idi, idj, FundamentEPP);

			//���ǲ���
			computeTrangulation(points_1, points_2, my_sfm_data, tuIndex1, tuIndex2,
				finalrt, points3DForGPS);

			//���ǲ�����ĵ�д�� my_sfm_data.s_root_path + /../ + points3D/
			write3DPoints(my_sfm_data, points3DForGPS, tuIndex1, tuIndex2);

			//����Ƕ�
			computeAngle(my_sfm_data, tuIndex1, tuIndex2, idi, idj, points3DForGPS, horizontalVecs, verticalAngles);
		}
	}
}



/*
*��Ƕ�ƽ��ֵ
*
*IO дtxt
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
	//����һ�����������
	for (size_t i = 0; i < horizontalVecs.size(); i++) {
		avgVec[0] += horizontalVecs[i][0];
		avgVec[1] += horizontalVecs[i][1];
	}
	//��ȡƽ��ˮƽ��
	avgHor = getShuiPing(avgVec[0], avgVec[1], 1.0, 0.0, 0.0, 0.0);
	//����ˮƽ�Ǳ�׼��
	double theta;
	for (size_t i = 0; i < horizontalVecs.size(); i++) {
		theta = (180.0 / M_PI) * acos((avgVec[0] * horizontalVecs[i][0] + avgVec[1] * horizontalVecs[i][1]) /
			(sqrt(avgVec[0] * avgVec[0] + avgVec[1] * avgVec[1])*sqrt(horizontalVecs[i][0] * horizontalVecs[i][0] + horizontalVecs[i][1] * horizontalVecs[i][1])));
		deviationHor += theta * theta;
	}
	deviationHor = sqrt(deviationHor / horizontalVecs.size());
	//���ƽ���Ƕ����ļ�
	string avgAngle = "angle/averageAngle.txt";
	fstream outAvgAngle(my_sfm_data.s_root_path + "/../" + avgAngle, ios::out);
	//�����Ϣ���ļ�
	outAvgAngle << setprecision(15) << "�����ǣ�" << avgVer << endl << "��׼�" << deviationVer << endl;
	outAvgAngle << setprecision(15) << "ˮƽ�ǣ�" << avgHor << endl << "��׼�" << deviationHor << endl;
	cout << "��������ˮƽ�ǵ�ƽ��ֵ��" << endl;
	cout << setprecision(15) << "�����ǣ�" << avgVer << endl << "��׼�" << deviationVer << endl;
	cout << setprecision(15) << "ˮƽ�ǣ�" << avgHor << endl << "��׼�" << deviationHor << endl;
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

	//��ȡGPS and ������Ϣ.......
	for (size_t i = 0; i < posesNum; ++i)
	{
		tmpGPS = allGPS[poseIndex[i]];
		longitude[i] = tmpGPS[0] * DEG_TO_RAD;
		latitude[i] = tmpGPS[1] * DEG_TO_RAD;
		altitude[i] = tmpGPS[2];
	}

	//��������������������
	double commonZ = altitude[0];
	for (size_t i = 0; i < posesNum; i++) {
		if (Count(altitude, posesNum, altitude[i]) < Count(altitude, posesNum, altitude[i + 1])) {
			commonZ = altitude[i + 1];
		}
	}

	projPJ pj_merc, pj_latlong;
	if (!(pj_merc = pj_init_plus("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) {
		/*cout << "ͶӰ����ʧ��!" << endl;*/
		exit(EXIT_FAILURE);
	}
	if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs"))) {
		/*cout << "ͶӰ����ʧ��!" << endl;*/
		exit(EXIT_FAILURE);
	}
	//��γ����ͶӰ����ϵ��ת��
	pj_transform(pj_latlong, pj_merc, posesNum, 1, longitude, latitude, NULL);
	//��γ������Ϊ0ʱ��ͶӰ����ں�С��С��ֵ���ڴ˺���
	for (size_t i = 0; i < posesNum; i++) {
		if (longitude[i] < 1e-5) {
			longitude[i] = 0.0;
		}
		if (latitude[i] < 1e-5) {
			latitude[i] = 0.0;
		}
	}
	//(3)��ȡ����ռ���ÿ�������X,Y,Z����
	vector<double> xCoor, yCoor, zCoor;
	for (openMVG::sfm::Poses::iterator iterP = my_sfm_data.poses.begin();
		iterP != my_sfm_data.poses.end(); ++iterP) {
		xCoor.push_back(iterP->second.center().x());
		yCoor.push_back(iterP->second.center().y());
		zCoor.push_back(iterP->second.center().z());
	}

	//(4)ʹ����С���˷�������ƽ��T������S
	//ʹ������֪ʶ����ƽ��T���Լ�����S�������û���������ƽ������Ϊ0.5m
	//����S
	Eigen::VectorXd x(3, 1);
	double gap = 0;
	for (size_t i = 0; i < posesNum - 1; i++) {
		gap = gap + sqrt((xCoor[i + 1] - xCoor[i])*(xCoor[i + 1] - xCoor[i]) +
			(yCoor[i + 1] - yCoor[i])*(yCoor[i + 1] - yCoor[i]));
	}
	gap = gap / (posesNum - 1);
	double S = 0.5 / gap;
	x(0) = S;
	//����T
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

	//(5)�����ǲ���������Ŀ��㣬����T,Sת�������GPS����
	string GPSfile = "GPS/GPS.txt";
	fstream outGPS(my_sfm_data.s_root_path + "/../" + GPSfile, ios::out);
	double x0 = points3DForGPS[0].x();
	double y0 = points3DForGPS[0].y();
	double z0 = points3DForGPS[0].z();
	x0 = x0 * x(0) + x(1);
	y0 = y0 * x(0) + x(2);
	//�������ƽ�������
	outGPS << "ͶӰƽ����������:" << endl;
	/*cout << "ͶӰƽ����������:" << endl;*/
	for (size_t i = 0; i < posesNum; i++) {
		/*cout << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;*/
		outGPS << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
	}
	delete[] longitude;
	delete[] latitude;
	delete[] altitude;
	/*cout << "Ŀ������" << ": " << x0 << " " << y0 << endl << endl;*/
	outGPS << "Ŀ������" << ": " << x0 << " " << y0 << endl << endl;
	//��ƽ��������ת��ΪGPS
	pj_transform(pj_merc, pj_latlong, 1, 1, &x0, &y0, NULL);

	//cout << setprecision(15) << "S: " << x[0] << endl;
	//cout << setprecision(15) << "Tx: " << x[1] << endl;
	//cout << setprecision(15) << "Ty: " << x[2] << endl << endl;
	outGPS << setprecision(15) << "S: " << x[0] << endl;
	outGPS << setprecision(15) << "Tx: " << x[1] << endl;
	outGPS << setprecision(15) << "Ty: " << x[2] << endl << endl;
	x0 *= RAD_TO_DEG;
	y0 *= RAD_TO_DEG;
	//������GPS����Ϊ0ʱ���д����
	if (x0 < 1e-3) {
		x0 = 0.0;
		y0 = 0.0;
	}
	outGPS << setprecision(15) << "���ȣ�" << x0 << endl;
	outGPS << setprecision(15) << "γ�ȣ�" << y0 << endl;
	//(6)���㺣��
	//��ȡ����ռ������������ľ�ֵ
	double sumOfZ = 0.0;
	for (size_t i = 0; i < posesNum; i++) {
		sumOfZ += zCoor[i];
	}
	sumOfZ /= posesNum;
	double Z = commonZ + (z0 - sumOfZ) * x[0];
	/*cout << setprecision(15) << "���Σ�" << Z << endl;*/
	outGPS << setprecision(15) << "���Σ�" << Z << endl;
	outGPS.close();
}


