#include "third_party\cmdLine\cmdLine.h"
#include "openMVG\cameras\cameras.hpp"
#include "openMVG\sfm\sfm.hpp"
#include "openmvg\geometry\rigid_transformation3D_srt.hpp"
#include "openMVG\cameras\Cameras_Common_command_line_helper.hpp"
#include "openMVG\system\timer.hpp"
#include "openMVG\multiview\triangulation_nview.hpp"

#include "opencv2/line_descriptor.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

#include "D:\code\ZTE\source\openMVG\src\software\SfM\tx.h"
#include "D:\code\ZTE\source\openMVG\src\software\SfM\capture.h"
#include "proj_api.h"
#include "D:\code\ZTE\source\openMVG\src\software\SfM\main_tx.h"

using namespace openMVG::sfm;

int main(int argc, char **argv) {

    CmdLine cmd;
    std::string sSfM_Data_Filename;
    double distanceBetweenTwoImages = 0.5;
    cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
    cmd.add(make_option('d', distanceBetweenTwoImages, "distanceBetweenTwoImages"));
    try {
        if (argc == 1) throw std::string("Invalid parameter.");
        cmd.process(argc, argv);
    }
    catch (const std::string& s) {
        std::cerr << "Usage: " << argv[0] << '\n'
            << "[-i|--input_file] path to a SfM_Data scene\n"
            << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    //*** 加载sfm_data
    SfM_Data my_sfm_data;
    if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
        std::cerr << std::endl
            << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }


    //*** Create directories
    string txtPath = my_sfm_data.s_root_path + "/../" + "txtFiles";
    mkdir(txtPath.c_str());
    string anglePath = my_sfm_data.s_root_path + "/../angle";
    mkdir(anglePath.c_str());
    string GPSDirPath = my_sfm_data.s_root_path + "/../" + "GPS";
    mkdir(GPSDirPath.c_str());

    // 获取虚拟空间的旋转矩阵
    size_t viewsNum = my_sfm_data.views.size();
    size_t posesNum = my_sfm_data.poses.size();
    vector<Eigen::Matrix<double, 3, 3>> rotations;
    vector<int> posesIndex;
    for (Poses::iterator itr = my_sfm_data.poses.begin(); itr != my_sfm_data.poses.end(); itr++) {
        rotations.push_back(itr->second.rotation().inverse());
        //存储有效poses的索引号
        posesIndex.push_back(itr->first);
    }

    // 获取现实空间中的旋转矩阵
    // 将手机矩阵转化为相机矩阵Remap
    string filePath = my_sfm_data.s_root_path;
    vector<Eigen::Matrix<double, 3, 3> > rotationsAndroid;
    vector<vector<double> > allGPS;
    readRotations(my_sfm_data, posesIndex, rotationsAndroid, allGPS, filePath);
    Eigen::Matrix<double, 3, 3> finalrt;
    virtualToReality(finalrt, posesNum, rotationsAndroid, rotations, my_sfm_data.s_root_path + "/..");


    vector<int> drawLinePicIndex;
#ifdef _WIN32
    std::cout << "SFM 成功的图像数量: " << posesIndex.size() << '\n';
    std::cout << "成功的图像索引号: ";
#endif
    for (size_t i = 0; i < posesIndex.size(); ++i) {
        //cout << posesIndex[i] << " ";
        drawLinePicIndex.push_back(posesIndex[i]);
    }
    int drawLinePicNum = posesIndex.size();
    vector<cv::Point2d> pointPair;
    // vector<vector<cv::line_descriptor::KeyLine> > keyLineArray;
    vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > > projPointMatVec;
    for (int i = 0; i < drawLinePicNum; ++i) {

        View *view = my_sfm_data.views.at(drawLinePicIndex[i]).get();
        //获取内外参
        openMVG::cameras::IntrinsicBase * cam = my_sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        openMVG::geometry::Pose3 pose = my_sfm_data.GetPoseOrDie(view);
        Eigen::Matrix<double, 3, 4> projMat = cam->get_projective_equivalent(pose);
        projPointMatVec.push_back(projMat);

        string img1Name = my_sfm_data.s_root_path + "/" + view->s_Img_path;

#ifdef _WIN32
        image_path = my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(drawLinePicIndex[i]) + ".jpg";
        capture(img1Name);
#else
        std::string image_path = my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(drawLinePicIndex[i]) + ".jpg";

#endif
        //从文件读截图的位置
        cv::Point2d pointOne;
        {
            std::fstream in(image_path + ".txt", std::ios::in);
            in >> pointOne.x >> pointOne.y;
            in.close();
        }
        pointPair.push_back(pointOne);
    }

    //输入框选天线图片的共同前缀名，此处是.../matches/picSmall0, 根据框选的的图片编号合成图片名
    //如.../matches/picSmall01.jpg
    //直线检测结果保存在keyLineArray
    //同时将画上直线的图片写到同一个目录， 如.../matches/picSmalee01_with_lines.jpg
    vector < vector < cv::line_descriptor::KeyLine> > keyLineArray;//保存直线提取结果
    std::vector<int> drawLinePair;// 保存每个小图片上选定的直线编号
    int validNumOfImgs = 0;
#ifdef _WIN32
    lineDetector(my_sfm_data.s_root_path + "/../matches/picSmall0", drawLinePicIndex, keyLineArray); // 直线提取
                                                                                                     //读取画好直线的小图片 显示用
    for (int i : drawLinePicIndex) {
        cv::Mat image = cv::imread(my_sfm_data.s_root_path + "/../matches/picSmall0" + to_string(i) + "_with_lines.jpg");
        if (!image.empty()) {
            string windowName = std::to_string(i);
            cv::namedWindow(windowName, 1);
            cv::imshow(windowName, image);
            cv::waitKey(0);
        }
    }

    //用户输入直线对的数量，并输入对应的编号，“-1”表示此处的直线没有被提取出来。
    std::cout << "输入直线在每张图像上的编号, 若某张图像上此直线没有提取出来，请输入‘-1’:" << endl;

    for (int j = 0; j < drawLinePicNum; j++) {
        int tmp;
        std::cin >> tmp;
        if (tmp > -1) {
            ++validNumOfImgs;
        }
        drawLinePair.push_back(tmp);
    }
    cv::destroyAllWindows();
#else
    /*******************        ANDROID         *****************/
    // 读取直线提取的结果 和 直线编号
    string smallPicPreName = my_sfm_data.s_root_path + "/../matches/picSmall0";
    for (auto i : drawLinePicIndex) {
        string smallPicName = smallPicPreName + to_string(i) + ".jpg";
        string picWithLine = smallPicPreName + to_string(i) + "_with_lines.jpg";
        vector<cv::line_descriptor::KeyLine> lineInOnePic;
        std::fstream in(picWithLine + ".txt", std::ios::in);
        int j = 0;
        while (!in.eof()) {
            cv::line_descriptor::KeyLine keyline;
            in >> keyline.startPointX >> keyline.startPointY
                >> keyline.endPointX >> keyline.endPointY;
            lineInOnePic.push_back(keyline);
        }
        in.close();
        keyLineArray.push_back(lineInOnePic);
    }
    {
        std::fstream pic_in(my_sfm_data.s_root_path + "/../matches/pic.txt", std::ios::in);
        int temp = -1;
        for (int j = 0; j < drawLinePicNum; j++) {
            pic_in >> temp;
            if (temp > -1)
                ++validNumOfImgs;
            drawLinePair.push_back(temp);
        }
    }

#endif //_WIN32

    /*****************   以下部分 应该不需要改变   ********************/
    //  角度输出在angle/averageAngle.txt
    //  位置输出在 GPS/GPS.txt
    /*********三、采用基于直线的三角测量方法计算直线的两个姿态角*******************/

    //1、获取图像索引号
    //drawLinePicIndex是SFM成功的图像的索引号，validImgLineIndex是SFM成功且成功提取出直线的图像索引号

    vector<int> validImgLineIndex;
    for (size_t i = 0; i != drawLinePair.size(); i++) {
        if (drawLinePair[i] != -1) {
            validImgLineIndex.push_back(drawLinePicIndex[i]);
        }
    }
    if (validImgLineIndex.size() < 3) {
        // cout << endl << "能够成功提取直线的图像必须大于或等于3" << endl;
        //system("pause");
        //return 0;
        exit(1);
    }
    //2、对图像上的直线进行去畸变操作
    //获取每个有效图像的直线上的始点和终点
    vector<vector<openMVG::Vec2 > > pointsForLines;
    for (size_t i = 0; i != drawLinePair.size(); i++) {
        cv::line_descriptor::KeyLine keyline = keyLineArray[i][drawLinePair[i]];
        double start_x = keyline.startPointX + pointPair[i].x;
        double start_y = keyline.startPointY + pointPair[i].y;
        openMVG::Vec2 start(start_x, start_y);
        double end_x = keyline.endPointX + pointPair[i].x;
        double end_y = keyline.endPointY + pointPair[i].y;
        openMVG::Vec2 end(end_x, end_y);

        vector<openMVG::Vec2 > tmp;
        tmp.push_back(start);
        tmp.push_back(end);
        pointsForLines.push_back(tmp);
    }

    //对所有有效图像上的点去畸变
    //去畸变的时候，必需使用的是原始的有效图像索引号，即SFM失败的图像也要包括其中，
    //因为此时SFM的结果保存了所有相机的内参信息，而去畸变使用的是内参，即使该相机pose计算失败，但是原始序号存储
    for (size_t i = 0; i < validImgLineIndex.size(); i++)
    {
        //获取虚拟空间中每一张图像的view，包含了图像，内参，外参
        View *view = my_sfm_data.views.at(validImgLineIndex[i]).get();
        //获取内外参
        openMVG::cameras::IntrinsicBase * cam = my_sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        //对原始提取的直线端点进行去畸变操作
        Eigen::Matrix<double, 2, 1> tmp;
        tmp = cam->get_ud_pixel(pointsForLines[i][0]);
        pointsForLines[i][0] = tmp;
        tmp = cam->get_ud_pixel(pointsForLines[i][1]);
        pointsForLines[i][1] = tmp;

    }


    //3、使用三焦张量计算直线姿态
    //获取多个三视角的组合
    vector<vector<int>> comb;
    //辅助容器
    vector<int> da;
    //获取图像索引号，注意此时并非使用原始的poses编号，而是组合的是所有Poses成功并且能够提取出直线的编号
    //因为去畸变需要原始索引号，去畸变结束后，所有数据都放置于编号统一的容器中，所以直接使用顺序的编号即可
    std::vector<int> validImgIdOfValidPoses;
    for (size_t i = 0; i < drawLinePair.size(); i++) {
        if (drawLinePair[i] != -1) {
            validImgIdOfValidPoses.push_back(i);
        }
    }
    //获取所有图像编号三子集的组合，构建三焦张量矩阵
    getcij(0, 3, validImgIdOfValidPoses, da, comb);
    std::vector<cv::Vec2d> horizontalVecs;
    std::vector<double> verticalAngles;
    for (int k = 0; k < comb.size(); k++) {
        //cout << comb[k][0] << " " << comb[k][1] << " " << comb[k][2] << endl;
        Eigen::MatrixXd W(3, 4);
        for (int i = 0; i < 3; i++) {
            Eigen::Matrix<double, 1, 4> tmpMat;
            Eigen::Matrix<double, 3, 1> tmpLine;
            //将平面的直线上的两个坐标转化为直线标准方程
            tmpLine = TwoPointsToABC(pointsForLines[comb[k][i]][0].x(), pointsForLines[comb[k][i]][0].y(), pointsForLines[comb[k][i]][1].x(), pointsForLines[comb[k][i]][1].y());
            tmpMat = tmpLine.transpose() * projPointMatVec[comb[k][i]];
            for (int j = 0; j < 4; j++) {
                W(i, j) = tmpMat[j];
            }
        }
        //对W进行奇异值分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svdW(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<double, 4, 1> v1, v2;
        for (int i = 0; i < 4; i++) {
            v1[i] = svdW.matrixV()(i, 2);
            v2[i] = svdW.matrixV()(i, 3);
        }
        Eigen::Matrix<double, 3, 1> v3, v4;
        for (int i = 0; i < 3; i++) {
            v3[i] = v1[i] / v1[3];
            v4[i] = v2[i] / v2[3];
        }
        //对空间点进行旋转
        v3 = finalrt*v3;
        v4 = finalrt*v4;
        double fuyang = getFuYang(v3[0], v3[1], v3[2], v4[0], v4[1], v4[2]);
        double shuiping = getShuiPing(v3[0], v3[1], v3[2], v4[0], v4[1], v4[2]);
        //cout << "俯仰角： " << fuyang << endl;
        //cout << "水平角： " << shuiping << endl;
        verticalAngles.push_back(fuyang);
        //计算水平矢量，以进行平均水平角和标准差的计算
        double vecX, vecY;
        //找到始点与终点，终点为z值更大的那个点
        if (v3[2] >= v4[2]) {
            vecX = v3[0] - v4[0];
            vecY = v3[1] - v4[1];
        }
        else {
            vecX = v4[0] - v3[0];
            vecY = v4[1] - v3[1];
        }
        //添加归一化的向量到容器中
        horizontalVecs.push_back(cv::Vec2d(vecX / (sqrt(vecX * vecX + vecY * vecY)), vecY / (sqrt(vecX * vecX + vecY * vecY))));

    }

    //计算俯仰角平均值与标准差
    double avgHor = 0.0, avgVer = 0.0, deviationHor = 0.0, deviationVer = 0.0;
    for (size_t i = 0; i < verticalAngles.size(); i++) {
        avgVer += verticalAngles[i];
    }
    avgVer /= verticalAngles.size();

    for (size_t i = 0; i < verticalAngles.size(); i++) {
        deviationVer += (verticalAngles[i] - avgVer) * (verticalAngles[i] - avgVer);
    }
    deviationVer = sqrt(deviationVer / verticalAngles.size());

    //计算水平角的平均值与标准差
    cv::Vec2d avgVec(0.0, 0.0);
    //将归一化的向量相加
    for (size_t i = 0; i < horizontalVecs.size(); i++) {
        avgVec[0] += horizontalVecs[i][0];
        avgVec[1] += horizontalVecs[i][1];
    }
    //获取平均水平角
    avgHor = getShuiPing(avgVec[0], avgVec[1], 1.0, 0.0, 0.0, 0.0);
    //计算水平角标准差
    double theta3;
    for (size_t i = 0; i < horizontalVecs.size(); i++) {
        theta3 = (180.0 / M_PI) * acos((avgVec[0] * horizontalVecs[i][0] + avgVec[1] * horizontalVecs[i][1]) /
            (sqrt(avgVec[0] * avgVec[0] + avgVec[1] * avgVec[1])*sqrt(horizontalVecs[i][0] * horizontalVecs[i][0] + horizontalVecs[i][1] * horizontalVecs[i][1])));
        deviationHor += theta3 * theta3;
    }
    deviationHor = sqrt(deviationHor / horizontalVecs.size());

    // cout << endl;
    // cout << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
    // cout << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;

    std::ofstream out_angle(anglePath + "/averageAngle.txt");
    out_angle << "俯仰角：" << avgVer << endl << "标准差：" << deviationVer << endl;
    out_angle << "水平角：" << avgHor << endl << "标准差：" << deviationHor << endl;
    out_angle.close();
    //system("pause");
    //mkdir((my_sfm_data.s_root_path + "/../" + "epipolarImg").c_str());

    /******四、直线测量无法计算出目标的空间点位置，因此使用同名相点的方式计算目标的空间坐标，以进行计算GPS*******/

    //此步骤与之前程序基本一致，计算GPS的方式也没有改变，不同之处在于仅仅使用一对双视角确定目标在SFM空间的位置
    //以及在程序的输入命令行中添加了对每两张图像的移动间隔的用户输入选项

    //(1)任意选取两个能够成功提取出直线的图像索引对，进行点对的三角测量
    int idi, idj, tuIndex1, tuIndex2;
    int flag = 0;
    for (size_t i = 0; i != drawLinePair.size(); i++) {
        if (drawLinePair[i] != -1 && flag == 0) {
            idi = i;
            tuIndex1 = drawLinePicIndex[i];
            flag++;
            //validImgLineIndex.push_back(drawLinePicIndex[i]);
        }
        else if (drawLinePair[i] != -1 && flag == 1) {
            idj = i;
            tuIndex2 = drawLinePicIndex[i];
            break;
            //validImgLineIndex.push_back(drawLinePicIndex[i]);
        }
    }

    //(2)计算两张图像的基础矩阵
    //第一种方法，通过两个相机的内外参进行计算
    View *view1 = my_sfm_data.views.at(tuIndex1).get();
    openMVG::cameras::IntrinsicBase * cam1 = my_sfm_data.GetIntrinsics().at(view1->id_intrinsic).get();
    openMVG::geometry::Pose3 pose1 = my_sfm_data.GetPoseOrDie(view1);

    View *view2 = my_sfm_data.views.at(tuIndex2).get();
    openMVG::cameras::IntrinsicBase * cam2 = my_sfm_data.GetIntrinsics().at(view2->id_intrinsic).get();
    openMVG::geometry::Pose3 pose2 = my_sfm_data.GetPoseOrDie(view2);

    Eigen::Matrix<double, 3, 4> proj1 = cam1->get_projective_equivalent(pose1),
        proj2 = cam2->get_projective_equivalent(pose2);
    openMVG::Vec3 c = my_sfm_data.poses[tuIndex1].center();

    cv::Mat promatric1(3, 4, CV_64FC1);
    cv::Mat promatric2(3, 4, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            promatric1.at<double>(i, j) = proj1(i, j);
            promatric2.at<double>(i, j) = proj2(i, j);
        }
    }
    //第一个相机中心
    cv::Mat C(4, 1, CV_64FC1);
    for (int i = 0; i < 3; i++) {
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
    cv::Mat FundamentEPP = eInvSym*promatric2*promatric1.inv(cv::DECOMP_SVD);

    //(3)设置极线图像的路径与图像名称，保留每次计算的极线图像
    //设置图像名称，注意\\仅在数组中占用一位
    string epiImg1Path = "epipolarImg/fisrtImg00.jpg";
    epiImg1Path[20] = tuIndex1 + '0';
    epiImg1Path[21] = tuIndex2 + '0';
    epiImg1Path = my_sfm_data.s_root_path + "/../" + epiImg1Path;
    string epiImg2Path = "epipolarImg/secondImg00.jpg";
    epiImg2Path[21] = tuIndex1 + '0';
    epiImg2Path[22] = tuIndex2 + '0';
    epiImg2Path = my_sfm_data.s_root_path + "/../" + epiImg2Path;


    //（4）计算、保存第一张图像的点，并在图像上画出
    string img1Name = my_sfm_data.s_root_path + "/" + view1->s_Img_path;
    cv::Mat imageMat1 = cv::imread(img1Name, 1);

    vector<openMVG::Vec2 > points_1, points_2;    //存入点，为后期进行三角测量做准备
    fstream out(txtPath + "/point.txt", ios::out);
    cv::line_descriptor::KeyLine keyline = keyLineArray[idi][drawLinePair[idi]];
    double start_x = keyline.startPointX + pointPair[idi].x;
    double start_y = keyline.startPointY + pointPair[idi].y;
    double end_x = keyline.endPointX + pointPair[idi].x;
    double end_y = keyline.endPointY + pointPair[idi].y;
    out << setprecision(15) << "##" << showpoint << start_x << " " << start_y << endl;
    out << setprecision(15) << "##" << showpoint << end_x << " " << end_y << endl;
    points_1.push_back(openMVG::Vec2(start_x, start_y));
    points_1.push_back(openMVG::Vec2(end_x, end_y));

    ////画图像1的直线和线段上始点和终点
    //line(imageMat1, cv::Point2d(start_x, start_y), cv::Point2d(end_x, end_y), cv::Scalar(255, 0, 0), 1);
    //for (size_t i = 0; i <= 2; i++) {
    //	circle(imageMat1, cv::Point2d(points_1[i].x(), points_1[i].y()), 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
    //}
    //imwrite(epiImg1Path, imageMat1);

    //(5)计算极线
    vector<cv::Vec3f> corresEpilines;
    openMVG::Mat3 FF;
    //把cv：：mat 转化为Eigen
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            FF(i, j) = FundamentEPP.at<double>(i, j);
    }
    for (size_t i = 0; i < points_1.size(); i++)
    {
        openMVG::Vec2 l_pt = points_1[i];
        openMVG::Vec3 line = FF*openMVG::Vec3(l_pt(0), l_pt(1), 1.0);
        cv::Vec3f coline(line(0), line(1), line(2));
        corresEpilines.push_back(coline);
    }


    //(6)画图2中的两点对应的两条极线并计算极线与直线的交点
    string img2Name = my_sfm_data.s_root_path + "/" + view2->s_Img_path;
    cv::Mat imageMat2 = cv::imread(img2Name, 1);
    //lineTp = 0;
    keyline = keyLineArray[idj][drawLinePair[idj]];
    float a2 = keyline.endPointY - keyline.startPointY;    //y2-y1
    float b2 = keyline.startPointX - keyline.endPointX;    //x1-x2
    float c2 = (keyline.endPointX + pointPair[idj].x)*(keyline.startPointY + pointPair[idj].y) - (keyline.startPointX + pointPair[idj].x)*(keyline.endPointY + pointPair[idj].y);    //x2y1-x1y2
                                                                                                                                                                                     //line(imageMat2, cv::Point(-c2 / a2, 0.0), cv::Point(-(b2*imageMat2.rows + c2) / a2, imageMat2.rows), cv::Scalar(255, 0, 0), 0.5);
                                                                                                                                                                                     //cv::circle(imageMat2, cv::Point(-(b2*imageMat2.rows + c2) / a2, imageMat2.rows), 20, cv::Scalar(0, 0, 255), 10, 8, 0);

    for (size_t k = 0; k < corresEpilines.size(); k++)
    {
        float a1 = corresEpilines[k][0];
        float b1 = corresEpilines[k][1];
        float c1 = corresEpilines[k][2];
        // draw the epipolar line between first and last column
        //line(imageMat2, cv::Point(0.0, -c1 / b1), cv::Point(imageMat2.cols, -(c1 + a1 * imageMat2.cols) / b1), cv::Scalar(0, 255, 0), 1.5);

        //计算交点
        cv::Point2d ans;
        ans.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
        ans.y = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
        out << setprecision(15) << ans.x << " " << ans.y << endl;
        points_2.push_back(openMVG::Vec2(ans.x, ans.y));
        //circle(imageMat2, ans, 0.5, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
    //imwrite(epiImg2Path, imageMat2);


    //(7)三角测量，并把测量后的点输出
    openMVG::Triangulation trianObj;
    std::vector<openMVG::Vec3> points3D;

    for (size_t i = 0; i < points_1.size(); i++) {
        //cout << "first camera's undistorted pixel coordinate:" << endl << cam1->get_ud_pixel(points_1[i]) << endl;
        //cout << "second camera's undistorted pixel coordinate:" << endl << cam2->get_ud_pixel(points_2[i]) << endl;
        trianObj.add(cam1->get_projective_equivalent(pose1), cam1->get_ud_pixel(points_1[i]));
        trianObj.add(cam2->get_projective_equivalent(pose2), cam2->get_ud_pixel(points_2[i]));
        //使用旋转转化矩阵，对三角测量后的顶点进行旋转转化，而不是对所有顶点、相机进行转化
        points3D.push_back(finalrt*trianObj.compute());
        trianObj.clear();
    }

    //将测量出的空间点写入文件
    string points3DPath = my_sfm_data.s_root_path + "/../" + "points3D";
    mkdir(points3DPath.c_str());
    string points3Dfile = "points3D/3Dpoints00.txt";
    points3Dfile[17] = tuIndex1 + '0';
    points3Dfile[18] = tuIndex2 + '0';
    fstream outPoints3D(my_sfm_data.s_root_path + "/../" + points3Dfile, ios::out);
    for (size_t i = 0; i < points3D.size(); i++) {
        outPoints3D << points3D[i].x() << " " << points3D[i].y() << " " << points3D[i].z() << " " << 255 << " " << 0 << " " << 0 << endl;
    }
    outPoints3D.close();


    //使用旋转转化矩阵，对所有其他点云进行旋转，因为要看到最后旋转后的点云姿态，在整个计算过程中，这一步是可选项
    /* for (Landmarks::iterator iterL = my_sfm_data.structure.begin();
    iterL != my_sfm_data.structure.end(); ++iterL) {
    iterL->second.X = finalrt*(iterL->second.X);
    }

    for (Poses::iterator iterP = my_sfm_data.poses.begin();
    iterP != my_sfm_data.poses.end(); ++iterP) {
    openMVG::geometry::Pose3 & pose = iterP->second;
    iterP->second.center() = finalrt*iterP->second.center();
    }
    */
    //-- Export to disk computed scene (data & visualizable results)
    //std::cout << "...Exp   ort SfM_Data to disk." << std::endl;

    //  Save(my_sfm_data,
    //    stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
    //     ESfM_Data(ALL));

    //  Save(my_sfm_data,
    //      stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
    //      ESfM_Data(ALL));



    /*******************目标GPS与海拔的计算***************/
    //(1)获取GPS与海拔信息
    vector<double> tmpGPS;
    double *longitude = new double[posesNum];
    double *latitude = new double[posesNum];
    double *altitude = new double[posesNum];
    //将有效的poses的横坐标，海拔输入
    for (size_t i = 0; i < posesNum; i++) {
        tmpGPS = allGPS[posesIndex[i]];
        longitude[i] = tmpGPS[0] * DEG_TO_RAD;
        latitude[i] = tmpGPS[1] * DEG_TO_RAD;
        altitude[i] = tmpGPS[2];
    }
    //获取海拔的众数
    double commonZ = altitude[0];
    for (size_t i = 0; i < posesNum; i++) {
        if (Count(altitude, posesNum, altitude[i]) < Count(altitude, posesNum, altitude[i + 1])) {
            commonZ = altitude[i + 1];
        }
    }

    //(2)将WGS84经纬度转化为墨卡托投影坐标系
    projPJ pj_merc, pj_latlong;
    if (!(pj_merc = pj_init_plus("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) {
        //cout << "投影创建失败!" << endl;
        exit(EXIT_FAILURE);
    }
    if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs"))) {
        //cout << "投影创建失败!" << endl;
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
    for (Poses::iterator iterP = my_sfm_data.poses.begin();
        iterP != my_sfm_data.poses.end(); ++iterP)
    {
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
    double S = distanceBetweenTwoImages / gap;
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
    string GPSPath = my_sfm_data.s_root_path + "/../" + "GPS";
    mkdir(GPSPath.c_str());
    string GPSfile = "GPS/GPS.txt";
    fstream outGPS(my_sfm_data.s_root_path + "/../" + GPSfile, ios::out);
    //使用双视角计算出来的两个目标三维坐标点信息，求平均确定目标在SFM空间的3D位置，相当于是取目标空间直线的中点
    openMVG::Vec3 targetPos = Eigen::Matrix<double, 3, 1>::Zero();
    for (size_t i = 0; i < points3D.size(); i++) {
        targetPos[0] += points3D[i][0];
        targetPos[1] += points3D[i][1];
        targetPos[2] += points3D[i][2];
    }
    for (int i = 0; i < 3; i++) {
        targetPos[i] /= points3D.size();
    }
    //cout << targetPos << endl;
    double x0 = targetPos.x();
    double y0 = targetPos.y();
    double z0 = targetPos.z();
    x0 = x0 * x(0) + x(1);
    y0 = y0 * x(0) + x(2);
    //输出所有平面坐标点
    outGPS << "投影平面的相机坐标:" << endl;
    //cout << "投影平面的相机坐标:" << endl;
    for (size_t i = 0; i < posesNum; i++) {
        //	cout << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
        outGPS << i << ": " << setprecision(15) << longitude[i] << " " << latitude[i] << endl;
    }
    delete[] longitude;
    delete[] latitude;
    delete[] altitude;
    //cout << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;
    outGPS << "目标坐标" << ": " << x0 << " " << y0 << endl << endl;
    //将平面坐标逆转化为GPS
    pj_transform(pj_merc, pj_latlong, 1, 1, &x0, &y0, NULL);
    // cout << endl;
    /*cout << setprecision(15) << "S: " << x[0] << endl;
    cout << setprecision(15) << "Tx: " << x[1] << endl;
    cout << setprecision(15) << "Ty: " << x[2] << endl << endl;*/
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
    //cout << setprecision(15) << "经度：" << x0 << endl;
    //cout << setprecision(15) << "纬度：" << y0 << endl;
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
    //cout << setprecision(15) << "海拔：" << Z << endl;
    outGPS << setprecision(15) << "海拔：" << Z << endl;
    outGPS.close();
    return EXIT_SUCCESS;
}
