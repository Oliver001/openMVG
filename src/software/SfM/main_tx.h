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
*@param_input  posesIndex 有效索引编号
*@param_output rAndroid 从txt文件读取的旋转矩阵
*@param_output allGPS 从txt文件读取的GPS
*@param_input filePath txt存放的根目录
*/
void readRotations(const SfM_Data & sfm_data,
  const vector<int> &posesIndex,
  vector<Eigen::Matrix<double, 3, 3> > &rAndroid,
  vector<vector<double> > &allGPS,
  string &filePath);

/*
*@param_out  out_finalrt	最终合成的旋转角
*@param_in	poseNum			pose数量
*@param_in rotationsAndroid 安卓得到的旋转角
*@param_in rotations sfm	得到的虚拟空间的旋转
*@param_in fileOutPath		角度结果写入的根目录
*/
void virtualToReality(Eigen::Matrix<double, 3, 3> &out_finalrt, int poseNum,
  const vector<Eigen::Matrix<double, 3, 3> > &rotationsAndroid,
  const vector<Eigen::Matrix<double, 3, 3> > &rotations,
  string fileOutPath);

/*
*@param_input smallPicPreName 小图片的共同前缀路径名称，my_Path/matches/picSmall0
*@param_input picIndex 进行框选的图片编号，与smallPicPreName共同组成小图片的全名, my_Path/mathces/picSmall01.jpg
*@param_output keyLineArray 直线检测结果保存的数组 返回值
*此函数内会进行图片的读取和写入 imread imwrite
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
*IO 写txt
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


void getcij(int numBe, int gNum, vector<int> r, vector<int> da, vector<vector<int>> & result);
Eigen::Matrix<double, 3, 1 > TwoPointsToABC(double x1, double y1, double x2, double y2);

void getPerij(int gNum, vector<int> r, vector<int> da, vector<vector<int>> & result);
#endif