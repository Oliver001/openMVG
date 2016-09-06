#pragma once
#ifndef _MAIN_TX_H
#define _MAIN_TX_H
#include "openMVG\sfm\sfm.hpp"
#include "openMVG\sfm\sfm_data.hpp"

#include "vector"
#include "string"
#include "opencv2\line_descriptor\descriptor.hpp"
#include "proj_api.h"
#include "tx.h"

using openMVG::sfm::SfM_Data;
using openMVG::sfm::Views;
using openMVG::sfm::View;
using std::vector;
using std::string;


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
    string &filePath);


/*
*@param_out  out_finalrt	���պϳɵ���ת��
*@param_in	poseNum			pose����
*@param_in rotationsAndroid ��׿�õ�����ת��
*@param_in rotations sfm	�õ�������ռ����ת
*@param_in fileOutPath		�ǶȽ��д��ĸ�Ŀ¼
*/
void virtualToReality(Eigen::Matrix<double, 3, 3> &out_finalrt, int poseNum,
	const vector<Eigen::Matrix<double, 3, 3> > &rotationsAndroid,
	const vector<Eigen::Matrix<double, 3, 3> > &rotations,
	string fileOutPath);


/*
*@param_input smallPicPreName СͼƬ�Ĺ�ͬǰ׺·�����ƣ�my_Path/matches/picSmall0
*@param_input picIndex ���п�ѡ��ͼƬ��ţ���smallPicPreName��ͬ���СͼƬ��ȫ��, my_Path/mathces/picSmall01.jpg
*@param_output keyLineArray ֱ�߼������������� ����ֵ
*�˺����ڻ����ͼƬ�Ķ�ȡ��д�� imread imwrite
*/
void lineDetector(const string &smallPicPreName, const vector<int> &picIndex,
	vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray);


/*
*@out FundamentEPP
*@in drawLinePicIndex
*@in tuIndex1
*@in tuIndex2
*@in my_sfm_data
*/
void computeFundament(cv::Mat &FundamentEPP, const vector<int> &drawLinePicIndex, int tuIndex1, int tuIndex2,
	openMVG::sfm::SfM_Data &my_sfm_data);


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
	cv::Mat FundamentEPP);


void computeTrangulation(const vector<openMVG::Vec2> &points_1, const vector<openMVG::Vec2> &points_2,
	openMVG::sfm::SfM_Data &my_sfm_data, int tuIndex1, int tuIndex2,
	Eigen::Matrix<double, 3, 3> finalrt,
	vector<openMVG::Vec3> &points3DForGPS);


void write3DPoints(openMVG::sfm::SfM_Data &my_sfm_data, const vector<openMVG::Vec3> &points3D,
	int tuIndex1, int tuIndex2);


void computeAngle(openMVG::sfm::SfM_Data &my_sfm_data,
	int tuIndex1, int tuIndex2,
	int idi, int idj,
	vector<openMVG::Vec3> &points3D,
	vector<cv::Vec2d> &horizontalVecs, vector<double> &verticalAngles);

void mdzz(openMVG::sfm::SfM_Data &my_sfm_data,
	Eigen::Matrix<double, 3, 3> finalrt,
	const vector<int> &drawLinePicIndex, const vector<int> &drawLinePair,
	const vector<vector<cv::line_descriptor::KeyLine> > &keyLineArray,
	vector<cv::Vec2d> &horizontalVecs, vector<double> &verticalAngles,
	vector<openMVG::Vec3> &points3DForGPS, const vector<cv::Point2d> &pointPair);

void computeAVG(const openMVG::sfm::SfM_Data &my_sfm_data,
	const vector<cv::Vec2d> &horizontalVecs, const vector<double> &verticalAngles);

void GPSandHeight(const vector<vector<double> > &allGPS, const vector<int> &poseIndex,
	openMVG::sfm::SfM_Data &my_sfm_data, const vector<openMVG::Vec3> &points3DForGPS);

#endif