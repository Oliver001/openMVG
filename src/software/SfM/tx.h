#pragma once
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Eigen\Eigen\Eigen"
#include <opencv2/highgui.hpp>
#include "opencv2/line_descriptor.hpp"

#ifdef __linux__
#include<sys/types.h>
#include<sys/stat.h>
#include<unistd.h>
#define mkdir(x) mkdir(x,0777)
#endif // __linux__

void RotationMatrixToEulerAnglesXYZ(Eigen::Matrix<double, 3, 3>&  R, double* euler);
void RotationMatrixToEulerAnglesZXY(Eigen::Matrix<double, 3, 3>& R, double *euler);
void RotationMatrixToEulerAnglesZYX(Eigen::Matrix<double, 3, 3>&  R, double* euler);
void getRotMatrixZXY(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getRotMatrixZYX(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getRotMatrixXYZ(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getInfoFromTxt(double* angles, std::string& path, std::string& identity);
void getRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path);
void getRemappedRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path);
double getDistance(double tx, double ty, double tz, double rx, double ry, double rz);
double getFuYang(double tx, double ty, double tz, double rx, double ry, double rz);
double getShuiPing(double tx, double ty, double tz, double rx, double ry, double rz);
bool sortdes(const cv::line_descriptor::KeyLine &k1, const cv::line_descriptor::KeyLine &k2);
int Count(double a[], int size, double x);
int drawLines(const char* imageName, const char* outputImgName);