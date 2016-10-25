#pragma once
#ifndef TX_MEASURE_H_
#define TX_MEASURE_H_

#include <string>
#include <vector>
#include <map>
#include <utility>
#include <iostream>
#include <fstream>
#include "Eigen/Eigen/Dense"
#include "openMVG/sfm/sfm.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/line_descriptor.hpp"

namespace TX {

	enum class DisplayType;
	struct Pose2Param;

	class TxMeasure {
	public:
		using Index = unsigned;
		TxMeasure(const std::string m_input_dir, const std::map<std::string, std::pair<cv::Point2d,cv::Point2d> > &m_lines) :input_file_name_(m_input_dir), lines_(m_lines) {}
		friend void f_DisplayInfomation(const TxMeasure &tx, DisplayType type);
		//double compute_angle_first(const Eigen::Vector3d &first, const Eigen::Vector3d &second);
		//double compute_angle_second(const Eigen::Vector3d &first, const Eigen::Vector3d &second);
		void d() { read_rotation_translation(); }
		void my_go(unsigned number_view);

		//const std::map<std::string, TxMeasure::Index> &d_get_image2poseId();
		//friend std::map<std::string, cv::line_descriptor::KeyLine> *d_CaptureLine(const TX::TxMeasure &tx);
	private:
		//
		openMVG::sfm::SfM_Data my_sfm_data_;
		//
		std::string input_file_name_;
		//
		std::string root_path_;

		//
		std::map <std::string, Index> image2poseId_;
		//
		std::map<Index, Eigen::Matrix3d> V2R_rotation_matrix_;
		std::map<Index, Eigen::Matrix3d> V2R_rotation_vector_;
		std::map<Index, Eigen::Matrix3d> V2R_rotations_;
		//
		std::map<Index, Eigen::Vector3d> translations_;

		std::map < std::string, std::pair < cv::Point2d, cv::Point2d> > lines_;

		std::map<Index, std::pair<Eigen::Vector3d, Eigen::Vector3d> > virtual_points_;

		std::map<Index, std::pair<Eigen::Vector3d, Eigen::Vector3d> > reality_points_;

		std::map<Index, std::vector<double> > GPS_;



		/*加载data 从image文件夹读取手机方向矩阵*/
		bool read_rotation_translation();


		//Eigen::Matrix3d compute_fundamental_matrix(TxMeasure::Index pose_a, TxMeasure::Index pose_b);
		Eigen::Matrix3d TxMeasure::compute_fundamental_matrix(TxMeasure::Index pose_a, TxMeasure::Index pose_b,
			const shared_ptr<openMVG::sfm::View> &view1, const shared_ptr<openMVG::cameras::IntrinsicBase> &cam1,
			const openMVG::geometry::Pose3 &pose1, const shared_ptr<openMVG::sfm::View> &view2,
			const shared_ptr<openMVG::cameras::IntrinsicBase> &cam2,
			const openMVG::geometry::Pose3 &pose2);



	/*	std::pair<openMVG::Vec2, openMVG::Vec2>  TxMeasure::compute_corresponding_point(TxMeasure::Index left, TxMeasure::Index right,
			const std::pair<cv::Point2d, cv::Point2d> &left_point, const std::pair<cv::Point2d, cv::Point2d> &right_point,
			const map<Index, Pose2Param>  &pre_info);*/
		std::pair<openMVG::Vec2, openMVG::Vec2>  TxMeasure::compute_corresponding_point(TxMeasure::Index left, TxMeasure::Index right,
			std::pair<cv::Point2d, cv::Point2d> left_point, std::pair<cv::Point2d, cv::Point2d> right_point,
			const map<Index, Pose2Param>  &pre_info);

		double TxMeasure::ComputeShuiPing_m(const vector<Eigen::Vector3d> &real_point,const string &name,const double mean);
		double TxMeasure::AvgShuiPing_m(vector<double> &angles, double r = 15);
		double ComputeShuiPing(const vector<Eigen::Vector3d> &real_point);
		double AvgShuiPing(vector<double> &angles, double r);
		double AvgCrossZero(vector<double> &angles, unsigned positive, unsigned negative, double width, double r);
		double ComputeFuyang(const vector<Eigen::Vector3d> &real_point);
		double AvgAngle(vector<double> &angles, double r);

		double AvgVector ( const vector<Eigen::Vector3d> &points);
		/*三角测量*/

		vector<Eigen::Vector3d> tranglation_2_view_rotation_matrix(map<Index, Pose2Param> &poseInfoForCorresponding);
		vector<Eigen::Vector3d> tranglation_2_view_rotation_vector(map<Index, Pose2Param> &poseInfoForCorresponding);

		vector<Eigen::Vector3d> tranglation_all_view_vector_matrix(map<Index, Pose2Param> &poseInfoForCorresponding);
		vector<Eigen::Vector3d> tranglation_all_view_vector_matrix_MI(map<Index, Pose2Param> &poseInfoForCorresponding);

	};

	void f_DisplayInfomation(const TxMeasure &tx, DisplayType type);
	//std::map<std::string, cv::line_descriptor::KeyLine> *d_CaptureLine(const TX::TxMeasure &tx);
}
#endif // !TX_MEASURE_H_
