#include "tx.h"
//�ֱ���XYZ, ZXY, ZYX��˳�򣬽���ת���ع�����ת�����������ת����ֽ�����ת��
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
//���ļ��л�ȡ������ת��
void getAngleFromTxt(double* angles, std::string& path, std::string& identity) {
  std::ifstream fin(path, std::ios::in);
  std::string line;
  while (fin >> line) {
    if (line == identity)
      for (int i = 0; i < 3; i++) {
        fin >> angles[i];
      }
    fin.close();
    return;
  }
}

// ��� RotationMatrix
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

// ��� RemappedRotationMatrix
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

// ���һ��·�������е� txt �ļ�
std::vector<std::string> listTxtFiles(std::string& path) {
  std::vector<std::string> txt_files;

  intptr_t  hFile = 0;
  struct _finddata_t fileInfo;
  std::string pathName, exdName;
  const char* p = pathName.assign(path).append("\\*").c_str();
  if ((hFile = _findfirst(p, &fileInfo)) == -1) {
    return txt_files;
  }
  do {
    const std::string fileName(fileInfo.name);
    if (fileName.length() > 3) {
      const std::string extension = fileName.substr(fileName.length() - 3, 3);
      if (extension == "txt") {
        txt_files.push_back(path + fileName);
      }
    }
  } while (_findnext(hFile, &fileInfo) == 0);

  _findclose(hFile);

  return txt_files;
}

//���һ��·�������е�jpg�ļ�
std::vector<std::string> listJpgFiles(std::string& path) {
  std::vector<std::string> txt_files;

  intptr_t  hFile = 0;
  struct _finddata_t fileInfo;
  std::string pathName, exdName;

  if ((hFile = _findfirst(pathName.assign(path).append("\\*").c_str(), &fileInfo)) == -1) {
    return txt_files;
  }
  do {
    const std::string fileName(fileInfo.name);
    if (fileName.length() > 3) {
      const std::string extension = fileName.substr(fileName.length() - 3, 3);
      if (extension == "jpg") {
        txt_files.push_back(path + fileName);
      }
    }

    //	cout << fileInfo.name << (fileInfo.attrib&_A_SUBDIR ? "[folder]" : "[file]") << endl;
  } while (_findnext(hFile, &fileInfo) == 0);

  _findclose(hFile);

  return txt_files;
}

//��ȡ����
double getDistance(double tx, double ty, double tz, double rx, double ry, double rz) {
  return (sqrt((tx - rx) * (tx - rx) + (ty - ry) * (ty - ry) + (tz - rz) * (tz - rz)));
}

//���㸩����
double getFuYang(double tx, double ty, double tz, double rx, double ry, double rz) {
  return asin(getDistance(tx, ty, tz, tx, ty, rz) / getDistance(tx, ty, tz, rx, ry, rz))*180.0 / M_PI;
}

//����ˮƽ��
double getShuiPing(double tx, double ty, double tz, double rx, double ry, double rz) {
  double angle;
  double vecX, vecY;
  //tzΪ�յ�������꣬rzΪʼ���������
  if (tz >= rz) {
    //��ȡ��ʸ��
    vecX = tx - rx;
    vecY = ty - ry;
    if (rx <= tx) {
      //t��r���ұߣ��յ���ʼ����ұ�
      angle = acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY)))* 180.0 / M_PI;
    } else {
      //t��r����ߣ��յ���ʼ������
      angle = 2 * M_PI - acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY)))* 180.0 / M_PI;
    }
    return angle;
  }
  //rzΪ�յ�������꣬tzΪʼ���������
  //��ȡ��ʸ��
  vecX = rx - tx;
  vecY = ry - ty;
  if (tx <= rx) {
    //r��t���ұߣ��յ���ʼ����ұ�
    angle = acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY)))* 180.0 / M_PI;
  } else {
    //r��t����ߣ��յ���ʼ������
    angle = 2 * M_PI - acos((0 * vecX + 1 * vecY) / (1 * sqrt(vecX * vecX + vecY * vecY)))* 180.0 / M_PI;
  }
  return angle;
}
