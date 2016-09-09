#ifndef __CAPTURE_H__
#define __CAPTURE_H__
#pragma once
#ifdef _WIN32
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <stdlib.h>
#include <opencv2/highgui.hpp>
#include "opencv2/line_descriptor.hpp"

using std::string;
using std::vector;
using std::cout;
using std::endl;
extern IplImage *src;
extern IplImage *img;
extern bool drawing;
extern CvRect rect;
extern CvPoint origin;
extern bool istuNumOne;
extern string image_path;

//获取鼠标点的位置
void onMouse(int event, int x, int y, int flags, void *param);
int capture(string& imageName);
#endif //ifdef _WIN32
#endif // !__CAPTURE_H__
