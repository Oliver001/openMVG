#include "capture.h"
#ifdef _WIN32
using std::string;
using std::vector;
using std::cout;
using std::endl;
IplImage *src;
IplImage *img;
bool drawing = false;
CvRect rect;
CvPoint origin;
bool istuNumOne = true;
string image_path = "";
//获取鼠标点的位置
void onMouse(int event, int x, int y, int flags, void *param) {
  if (drawing) {
    rect.x = MIN(origin.x, x);
    rect.y = MIN(origin.y, y);
    rect.width = abs(origin.x - x);
    rect.height = abs(origin.y - y);
  }

  if (event == CV_EVENT_LBUTTONDOWN && !CV_EVENT_MOUSEMOVE) {
    drawing = true;
    origin = cvPoint(x, y);
    rect = cvRect(x, y, 0, 0);
  } else if (event == CV_EVENT_LBUTTONUP) {
    drawing = false;
    if (rect.height == 0 || rect.width == 0) {
      //cvDestroyWindow("ScreenShot");
      return;
    }
    IplImage* img = cvCreateImage(cvSize(rect.width, rect.height), src->depth, src->nChannels);
    cout << rect.x << " " << rect.y << " " << rect.height << " " << rect.width << endl;
    cvSetImageROI(src, rect);
    cvCopy(src, img, 0);
    cvResetImageROI(src);
    cvSaveImage(image_path.c_str(), img);
    cvReleaseImage(&img);
    return;
  }
}

int capture(string& imageName) {
  src = cvLoadImage(imageName.c_str());
  if (!src) {
    printf("Could not load image file: %s\n", imageName.c_str());
    return EXIT_FAILURE;
  }
  cvNamedWindow("screenshot", 0);
  //缩小提取的区域
  cvSetMouseCallback("screenshot", onMouse, NULL);//捕捉鼠标
  cvShowImage("screenshot", src);
  cvWaitKey(0);
  cvReleaseImage(&src);
  if (rect.height > 0 && rect.width > 0) {
    std::fstream out(image_path + ".txt", std::ios::out);
    out << rect.x << " " << rect.y << endl;
    out.close();
  }
  return EXIT_SUCCESS;
}
#endif