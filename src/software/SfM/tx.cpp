#include "tx.h"
//分别以XYZ, ZXY, ZYX的顺序，将旋转角重构成旋转矩阵或者由旋转矩阵分解至旋转角
void RotationMatrixToEulerAnglesXYZ(Eigen::Matrix<double, 3, 3>&  R, double* euler) {
  double r00 = R(0, 0), r01 = R(0, 1), r02 = R(0, 2), r11 = R(1, 1), r12 = R(1, 2), r22 = R(2, 2);
  euler[0] = atan2(-r01, r00) * 180.0 / M_PI;
  euler[1] = atan2(r02, sqrt(r00*r00 + r01*r01)) * 180.0 / M_PI;
  euler[2] = atan2(-r12, r22) * 180.0 / M_PI;
}

void RotationMatrixToEulerAnglesZXY(Eigen::Matrix<double, 3, 3>& R, double *euler) {
  euler[0] = -atan2(R(0, 1), R(1, 1)) * 180.0 / M_PI;
  euler[1] = atan2(-R(2, 0), R(2, 2)) * 180.0 / M_PI;
  euler[2] = -asin(-R(2, 1)) * 180.0 / M_PI;
}

void RotationMatrixToEulerAnglesZYX(Eigen::Matrix<double, 3, 3>&  R, double* euler) {
  double r21 = R(2, 1), r22 = R(2, 2);
  euler[0] = atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
  euler[1] = atan2(-R(2, 0), sqrt(r21*r21 + r22*r22)) * 180.0 / M_PI;
  euler[2] = atan2(r21, r22) * 180.0 / M_PI;
}

void getRotMatrixZXY(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX) {
  angleZ = angleZ * M_PI / 180.0;
  angleX = angleX * M_PI / 180.0;
  angleY = angleY * M_PI / 180.0;
  Eigen::Matrix<double, 3, 3> rotX, rotY, rotZ;
  rotZ << cos(angleZ), -sin(angleZ), 0, sin(angleZ), cos(angleZ), 0, 0, 0, 1;
  rotX << 1, 0, 0, 0, cos(angleX), -sin(angleX), 0, sin(angleX), cos(angleX);
  rotY << cos(angleY), 0, sin(angleY), 0, 1, 0, -sin(angleY), 0, cos(angleY);
  R = rotZ*rotX*rotY;
}

void getRotMatrixZYX(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX) {
  angleZ = angleZ * M_PI / 180.0;
  angleX = angleX * M_PI / 180.0;
  angleY = angleY * M_PI / 180.0;
  Eigen::Matrix<double, 3, 3> rotX, rotY, rotZ;
  rotZ << cos(angleZ), -sin(angleZ), 0, sin(angleZ), cos(angleZ), 0, 0, 0, 1;
  rotX << 1, 0, 0, 0, cos(angleX), -sin(angleX), 0, sin(angleX), cos(angleX);
  rotY << cos(angleY), 0, sin(angleY), 0, 1, 0, -sin(angleY), 0, cos(angleY);
  R = rotZ*rotY*rotX;
}

void getRotMatrixXYZ(Eigen::Matrix<double, 3, 3>&  R, double angleZ, double angleY, double angleX) {
  angleZ = angleZ * M_PI / 180.0;
  angleX = angleX * M_PI / 180.0;
  angleY = angleY * M_PI / 180.0;
  Eigen::Matrix<double, 3, 3> rotX, rotY, rotZ;
  rotZ << cos(angleZ), -sin(angleZ), 0, sin(angleZ), cos(angleZ), 0, 0, 0, 1;
  rotX << 1, 0, 0, 0, cos(angleX), -sin(angleX), 0, sin(angleX), cos(angleX);
  rotY << cos(angleY), 0, sin(angleY), 0, 1, 0, -sin(angleY), 0, cos(angleY);
  R = rotX*rotY*rotZ;
}
//从文件中获取三个旋转角
void getInfoFromTxt(double* angles, std::string& path, std::string& identity) {
  std::ifstream fin(path, std::ios::in);
  std::string line;
  while (fin >> line) {
    if (line == identity) {
      for (int i = 0; i < 3; i++) {
        fin >> angles[i];
      }//for
      fin.close();
      return;
    }//if
  }//while
}

// 获得 RotationMatrix
void getRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path) {
  std::ifstream fin(path, std::ios::in);
  std::string line;
  while (fin >> line) {
    if (line == "RotationMatrixFromVector:") {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          fin >> R(i, j);
        }//for j
      }//for i
      fin.close();
      return;
    }//if
  }//while
}

// 获得 RemappedRotationMatrix
void getRemappedRotationMatrix(Eigen::Matrix<double, 3, 3>& R, std::string& path) {
  std::ifstream fin(path, std::ios::in);
  std::string line;
  while (fin >> line) {
    if (line == "RemappedRotationMatrixFromVector:") {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          fin >> R(i, j);
        }//for j
      }//for i
      fin.close();
      return;
    }//if
  }//while
}

//获取距离
double getDistance(double tx, double ty, double tz, double rx, double ry, double rz) {
  return (sqrt((tx - rx) * (tx - rx) + (ty - ry) * (ty - ry) + (tz - rz) * (tz - rz)));
}

//计算俯仰角
double getFuYang(double tx, double ty, double tz, double rx, double ry, double rz) {
  return (90.0 - asin(getDistance(tx, ty, tz, tx, ty, rz) / getDistance(tx, ty, tz, rx, ry, rz))*180.0 / M_PI);
}

//计算水平角
double getShuiPing(double tx, double ty, double tz, double rx, double ry, double rz) {
  double angle;
  double vecX, vecY;
  //tz为终点的z坐标，rz为始点的z坐标
  if (tz >= rz) {
    //获取到矢量
    vecX = tx - rx;
    vecY = ty - ry;
    if (rx <= tx) {
      //t在r的右边，终点在始点的右边
      angle = (acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY))))* 180.0 / M_PI;
    } else {
      //t在r的左边，终点在始点的左边
      angle = (2 * M_PI - acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY))))* 180.0 / M_PI;
    }
    return angle;
  }
  //rz为终点的z坐标，tz为始点的z坐标
  //获取到矢量
  vecX = rx - tx;
  vecY = ry - ty;
  if (tx <= rx) { 
    //r在t的右边，终点在始点的右边
    angle = (acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY))))* 180.0 / M_PI;
  } else {
    //r在t的左边，终点在始点的左边
    angle = (2 * M_PI - acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY))))* 180.0 / M_PI;
  }
  return angle;
}
//比较keyLine
bool sortdes(const cv::line_descriptor::KeyLine &k1, const cv::line_descriptor::KeyLine &k2) {
  return k1.lineLength > k2.lineLength;
}

int Count(double a[], int size, double x)
{
	int i, frequency = 0;
	for (i = 0; i<size; i++)
	{
		//double数的判断不能使用等于
		if (abs(a[i] - x) <= 0.1)
			frequency++;
	}
	return frequency;
}

int drawLines(const char* imageName, const char* outputImgName) {
  std::vector<cv::line_descriptor::KeyLine> keylines;
	//(3)从路径中读取框选后的小图像，以进行直线提取
	cv::Mat imgMat1 = cv::imread(imageName, 1);
	if (imgMat1.data == NULL) {
		std::cout << "Error, images could not be loaded. Please, check their path" << std::endl;
		keylines.clear();
		return -1;
	}
	cv::Mat mask = cv::Mat::ones(imgMat1.size(), CV_8UC1);
	cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
	bd->detect(imgMat1, keylines, mask);

	///////*选择所需线并显示*/
	sort(keylines.begin(), keylines.end(), sortdes);
	size_t limitMax = 10 < keylines.size() ? 10 : keylines.size();
	for (int i = 0; i < limitMax; i++) {
		line(imgMat1, CvPoint(keylines[i].startPointX, keylines[i].startPointY), CvPoint(keylines[i].endPointX, keylines[i].endPointY), cv::Scalar(0, 0, 255), 1);
		std::string id = "0";
		id[0] = '0' + i;
		putText(imgMat1, id, CvPoint((keylines[i].startPointX + keylines[i].endPointX) / 2, (keylines[i].startPointY + keylines[i].endPointY) / 2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
	}
	imwrite(outputImgName, imgMat1);
	std::string txtpath = outputImgName;
	std::fstream out(txtpath + ".txt", std::ios::out);
	for (size_t i = 0; i < limitMax; i++) {
	  out << keylines[i].startPointX << " " << keylines[i].startPointY << " "
	    << keylines[i].endPointX << " " << keylines[i].endPointY << std::endl;
	}
	out.close();
	return keylines.size();
}