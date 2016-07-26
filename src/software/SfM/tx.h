#pragma once
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <io.h>
#include <string>
#include "Eigen\Eigen\Eigen"

void RotationMatrixToEulerAnglesXYZ(Eigen::Matrix<double, 3, 3>&  R, double* euler);
void RotationMatrixToEulerAnglesZXY(Eigen::Matrix<double, 3, 3>& R, double *euler);
void RotationMatrixToEulerAnglesZYX(Eigen::Matrix<double, 3, 3>&  R, double* euler);
void getRotMatrixZXY(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getRotMatrixZYX(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getRotMatrixXYZ(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX);
void getAngleFromTxt(double* angles, std::string& path, std::string& identity);
void getRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path);
void getRemappedRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path);
std::vector<std::string> listTxtFiles(std::string& path);
std::vector<std::string> listJpgFiles(std::string& path);
double getDistance(double tx, double ty, double tz, double rx, double ry, double rz);
double getFuYang(double tx, double ty, double tz, double rx, double ry, double rz);
double getShuiPing(double tx, double ty, double tz, double rx, double ry, double rz);
