#include <map>
#include <vector>
#include <array>
#include <utility>
#include "Eigen\Eigen\Dense"
#include "opencv2\calib3d.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "myIOutility.h"
#include "tx_measure.h"
#include "openMVG/multiview/triangulation_nview.hpp"
#include <algorithm>
namespace TX {
	using  namespace openMVG::sfm;
	using namespace openMVG::cameras;
	using namespace openMVG::geometry;
	using cv::Mat;
	using std::vector;

	double getDistance(double tx, double ty, double tz, double rx, double ry, double rz) {
		return (sqrt((tx - rx) * (tx - rx) + (ty - ry) * (ty - ry) + (tz - rz) * (tz - rz)));
	}

	/*	two pairs of (x,y,z) to compute incline angle
	*/
	double getFuYang(double tx, double ty, double tz, double rx, double ry, double rz) {
		//std::cout << "fuyang" << tx << ' ' << ty << ' ' << tz << ' ' << rx << ' ' << ry << ' ' << rz << std::endl;
		return (90.0 - asin(getDistance(tx, ty, tz, tx, ty, rz) / getDistance(tx, ty, tz, rx, ry, rz))*180.0 / M_PI);
	}
	
	/* use k-means(k = 1 here) to compute average angle
	*/
	double TxMeasure::AvgAngle(vector<double> &angles,double r = 3) {
		std::cout << "\ninit size: " << angles.size() << '\n';
		double sum = 0;
		for (const auto&k : angles) {
			sum += k;
		}
		sum /= angles.size();
		std::sort(angles.begin(), angles.end());
		if (angles.back() <= 10 || std::abs(angles.front() - angles.back()) < 10) {
			r = 3;
		}
		std::cout << "MAX: " << angles.back() << " MIN: "<< angles.front() << '\n';
		while (std::abs(angles.front() - angles.back()) >= 2*r) {
			std::cout << "small:" << angles.front() << ' ' << "large:" << angles.back() << '\n';
			double front_r = std::abs(angles.front() - sum);
			double back_r = std::abs(angles.back() - sum);
			if (front_r > back_r) {
				auto iter = angles.begin();
				angles.erase(iter);
			}
			else {
				angles.pop_back();
			}
			sum = 0;
			for (const auto &k : angles) {
				sum += k;
			}
			sum /= angles.size();
		}
		std::cout << "angles.size():" << angles.size() << '\n';
		std::cout << "MAX: " << angles.back() << " MIN: " << angles.front() << '\n';
		return sum;
	}

	/*	compute incline angle using (x,y,z) points from backprojective;
	*/
	double TxMeasure::ComputeFuyang(const vector<Eigen::Vector3d> &real_point) {
		vector<double> result_angle;
		result_angle.reserve(real_point.size() / 2);
		for (unsigned i = 0; i < real_point.size(); i += 2) {
			double angle = getFuYang(real_point[i](0), real_point[i](1), real_point[i](2),
				real_point[i + 1](0), real_point[i + 1](1), real_point[i + 1](2));
			result_angle.push_back(angle);
		}
		double average_angle_fuyang_rotation = AvgAngle(result_angle);
		return average_angle_fuyang_rotation;
	}

	/*	use two pair of (x,y,z) to compute single horizon angle
	*/
	double getShuiPing(double tx, double ty, double tz, double rx, double ry, double rz) {
		/*	consider the point with smaller z value as origin point
			!!!actually not proper, because only a line can not represent the orientation of a plane!!!
		*/
		double x, y;
		if (tz > rz) {
			x = tx - rx;
			y = ty - ry;
		}
		else {
			x = rx - tx;
			y = ry - ty;
		}
		double angle = y / std::sqrt(x*x + y*y);
		angle = acos(angle)* 180 /M_PI;
		if (x < 0) {
			angle = 360 - angle;
		}
		return angle;
	}

	/*	*use histogram to get the main orientation of a set of anlges
	*/
	void HistogramCount(const vector<double> &angles,double width, vector<unsigned> &angle_histogram) {
		unsigned bins = 360 / width;
		for (const auto&k : angles) {
			unsigned bin = k / width;
			angle_histogram[bin] += 1;
			angle_histogram[(bin + 1)%bins] += 1;
		}
	}

	/*	*handle the situation when main orientation cross zero
	*/
	double TxMeasure::AvgCrossZero(vector<double> &angles, unsigned positive, unsigned negative,double width, double r = 5) {
		if (positive > negative) {
			for (auto &k : angles) {
				if (k >= 360 - width) {
					k = 360 - k;
				}
			}
		}
		else {
			for (auto &k : angles) {
				if (k <= width) {
					k += 360;
				}
			}
		}
		double result = AvgAngle(angles, r);
		return result;
	}

	double TxMeasure::AvgShuiPing(vector<double> &angles, double r = 15) {
		std::sort(angles.begin(), angles.end());
		unsigned bin_number = 360 / r;
		vector<unsigned> angle_histogram(bin_number, 0);
		HistogramCount(angles, r, angle_histogram);

		unsigned max_bin = 0;
		unsigned max_element = angle_histogram.at(0);
		double angle_horizon = 0;
		for (unsigned i = 0; i < bin_number; ++i) {
			if (angle_histogram[i] > max_element) {
				max_bin = i;
			}
		}
		if (max_bin != 0) {
			vector<double> angle_in_bin;
			angle_in_bin.reserve(angle_histogram[max_bin]);
			for (const auto &k : angles) {
				if (k >= (max_bin - 1) * r && k <= (max_bin + 1)*r) {
					angle_in_bin.push_back(k);
				}
			}
			angle_horizon = AvgAngle(angles,7);
		}
		else {
			vector<double> angle_in_bin;
			unsigned positive = 0, negative = 0;
			angle_in_bin.reserve(angle_histogram[max_bin]);
			for (const auto &k : angles) {
				if (k < r) {
					++positive;
					angle_in_bin.push_back(k);
				}
				else if (k >= 360 - r) {
					++negative;
					angle_in_bin.push_back(k);
				}
			}
			angle_horizon =  AvgCrossZero(angles, positive, negative, r);
		}
		return angle_horizon;
	}
	double TxMeasure::AvgShuiPing_m(vector<double> &angles, double r) {
		//std::sort(angles.begin(), angles.end());
		return AvgAngle(angles, 7.5);
	}
	/*	*compute horizontal angle from a set points
	*/
	double TxMeasure::ComputeShuiPing(const vector<Eigen::Vector3d> &real_point) {

		vector<double> result_angle;
		result_angle.reserve(real_point.size() / 2);
		for (unsigned i = 0; i < real_point.size(); i += 2) {
			double angle = getShuiPing(real_point[i](0), real_point[i](1), real_point[i](2),
				real_point[i + 1](0), real_point[i + 1](1), real_point[i + 1](2));
			result_angle.push_back(angle);
		}


		std::sort(result_angle.begin(), result_angle.end());

		return AvgShuiPing_m(result_angle);
	}

	double StdDeviation(const vector<double> &angles, const double mean) {
		double distance_sum = 0;
		unsigned n = angles.size();
		for (const auto &k : angles) {
			distance_sum += (k - mean)*(k - mean);
		}

		return std::sqrt(distance_sum / n);
	}

	double TxMeasure::ComputeShuiPing_m(const vector<Eigen::Vector3d> &real_point,const string &name,const double mean) {

		vector<double> result_angle;
		result_angle.reserve(real_point.size() / 2);
		for (unsigned i = 0; i < real_point.size(); i += 2) {
			double angle = getShuiPing(real_point[i](0), real_point[i](1), real_point[i](2),
				real_point[i + 1](0), real_point[i + 1](1), real_point[i + 1](2));
			result_angle.push_back(angle);
		}
		std::ofstream out(root_path_ + "/../"+name, std::ios_base::out);

		for (const auto& k : result_angle) {
			out << k << '\n';
		}
		
		std::sort(result_angle.begin(), result_angle.end());
	//	vector<double> a;
		//double down = result_angle.back() - result_angle.front();
		//down = result_angle.back() - down *2 /3;
		//for (auto iter = result_angle.crbegin(); iter != result_angle.crend(); ++iter) {
		//	if (*iter >= down)
		//		a.push_back(*iter);
	//		else
	//			break;
//
	//	}
		
		double s = StdDeviation(result_angle, mean);
		out << "std: " << s << '\n';
		out << "end~~~~~~~~~~~~~~~~~~~~~~~~~~~fgh\n";
		return s;
		//return AvgShuiPing_m(result_angle);
	}


	void getRotZXY(double z, double x, double y, Eigen::Matrix3d &out) {
		double c1 = std::cos(z);
		double s1 = std::sin(z);
		double c2 = std::cos(x);
		double s2 = std::sin(x);
		double c3 = std::cos(y);
		double s3 = std::sin(y);
		out(0, 0) = c1*c3 - s1*s2*s3;
		out(0, 1) = -c2*s1;
		out(0, 2) = c1*s3 + c3*s1*s2;
		out(1, 0) = c3*s1 + c1*s2*s3;
		out(1, 1) = c1*c2;
		out(1, 2) = s1*s3 - c1*c3*s2;
		out(2, 0) = -c2*s3;
		out(2, 1) = s2;
		out(2, 2) = c2*c3;
		Eigen::Matrix3d r;
		r << 0, 1, 0, 1, 0, 0, 0, 0, -1;
		out *= r;
	}

	/*	*read infomation from sfm_data.json file and txt files contain phones' orientations
	*/
	bool TxMeasure::read_rotation_translation() {

		/* Load sfm_data.json from Global_Reconstruction directory */
		if (!openMVG::sfm::Load(my_sfm_data_, input_file_name_, ESfM_Data(INTRINSICS|VIEWS|EXTRINSICS))) {
			return false;
		}

		/* assign member value root_path */
		root_path_ = my_sfm_data_.s_root_path;
		const Poses &pose = my_sfm_data_.poses;
		const Views &view = my_sfm_data_.views;

		/*	two temporary container.
			first for valid pose id and its rotation matrix in sfm frame,
			second one for rotation in virtual space which matches the id
			and insert the id and its translation into class member translations.
		*/
		std::map<Index, Eigen::Matrix3d> rotation_in_pose;
		for (auto it = pose.cbegin(); it != pose.cend(); ++it) {
			rotation_in_pose.insert({ it->first, it->second.rotation() });
			translations_.insert({ it->first, it->second.translation() });
		}

		/*  1.  read GPS and rotation matrix from the file which the pose id and its filename correspond,
			2.  compute back matrix from the virtual frame to the world frame
				using formula MATRIXv2r = inverse(MATRIXworld2phone) * MATRIXvirtual2phone
			3.  store the GPS infomation into class member GPS_ with its corresponding pose id
				store the final matrices into V2R_rotations_ with its corresponding pose id
		*/

		std::map<Index, Eigen::Vector3d> tmp_total_first;
		std::map<Index, Eigen::Vector3d> tmp_total_second;

		for (auto it = rotation_in_pose.cbegin(); it != rotation_in_pose.cend(); ++it) {
			//const std::string image_name = my_sfm_data_.views.at(it->first)->s_Img_path;
			if (it->first != view.at(it->first)->id_pose)
				continue;
			const std::string image_name = my_sfm_data_.views.at(it->first)->s_Img_path;
			const std::string image_txt = image_name + ".txt";
			const std::string image_dir_name = root_path_ + image_txt;
			std::vector<double> parameter_in_txt(12, 0);
			ReadSensorTxt(image_dir_name, parameter_in_txt, SensorType::RemappedRotation);
			GPS_.insert({ it->first, std::vector<double>({parameter_in_txt[0],parameter_in_txt[1],parameter_in_txt[2]}) });
			Eigen::Matrix3d rotation_matrix_in_txt;
			rotation_matrix_in_txt << parameter_in_txt[3], parameter_in_txt[4], parameter_in_txt[5],
				parameter_in_txt[6], parameter_in_txt[7], parameter_in_txt[8],
				parameter_in_txt[9], parameter_in_txt[10], parameter_in_txt[11];
			//Eigen::Matrix3d rotation_virtual2reality = rotation_matrix_in_txt * it->second;//.inverse();
			//Eigen::Matrix3d rotation_virtual2reality = rotation_matrix_in_txt;
			Eigen::Matrix3d rotation_virtual2reality = rotation_matrix_in_txt;
			V2R_rotations_.insert({ it->first, rotation_virtual2reality });
			image2poseId_.insert({ image_name.substr(1),it->first });

			Eigen::Matrix3d pose_rotation = it->second;
			Eigen::Matrix3d total_rotation = rotation_virtual2reality * pose_rotation;


			/*************************************************************************/
			Eigen::Matrix3d rot;
			rot << 0, 1, 0, 1, 0, 0, 0, 0, -1;
			{
				std::vector<double> param_2(12,0);
				ReadSensorTxt(image_dir_name, param_2, SensorType::RotationMatrixFromVector);
				Eigen::Matrix3d rotation_matrix_1;
				rotation_matrix_1 << param_2[3], param_2[4], param_2[5],
					param_2[6], param_2[7], param_2[8],
					param_2[9], param_2[10], param_2[11];
				Eigen::Matrix3d r = rotation_matrix_1*rot;
				V2R_rotation_vector_.insert({ it->first,r });
				
			}
			/*************************************************************************/
			
		}
		return true;
	}

	/*	*here we use the projective matrix estimated by sfm
		not use correnponding features method in opencv with RANSAC in corresponding points.
		!!!So fundamental matrix's precision is depend on sfm reconstrucion precision.
	*/
	Eigen::Matrix3d TxMeasure::compute_fundamental_matrix(TxMeasure::Index pose_a, TxMeasure::Index pose_b,
		const shared_ptr<openMVG::sfm::View> &view1, const shared_ptr<openMVG::cameras::IntrinsicBase> &cam1,
		const openMVG::geometry::Pose3 &pose1, const shared_ptr<openMVG::sfm::View> &view2,
		const shared_ptr<openMVG::cameras::IntrinsicBase> &cam2,
		const openMVG::geometry::Pose3 &pose2) {

		//const auto &view1 = my_sfm_data_.views.at(pose_a);
		//const auto &cam1 = my_sfm_data_.intrinsics.at(view1->id_intrinsic);
		//const auto &pose1 = my_sfm_data_.GetPoseOrDie(view1.get());
		//const auto &view2 = my_sfm_data_.views.at(pose_b);
		//const auto &cam2 = my_sfm_data_.intrinsics.at(view2->id_intrinsic);
		//const auto &pose2 = my_sfm_data_.GetPoseOrDie(view2.get());

		Eigen::Matrix<double, 3, 4> proj_a = cam1->get_projective_equivalent(pose1);
		Eigen::Matrix<double, 3, 4> proj_b = cam2->get_projective_equivalent(pose2);

		//std::cout << pose_a  << ":"<<  pose1.rotation() << '\n';
		//std::cout << pose_b << ":" << pose2.rotation() << '\n';
		const auto &center_a = my_sfm_data_.poses[pose_a].center();
		//std::cout << pose_a << "center: " << center_a << '\n';
		Eigen::Vector4d C_a;
		C_a << center_a(0), center_a(1), center_a(2), 1;

		Eigen::Vector3d ee = proj_b * C_a;
		Eigen::Matrix3d eInvSym = Eigen::Matrix3d::Zero();
		eInvSym << 0, -ee(2), ee(1),
			ee(2), 0, -ee(0),
		-ee(1), ee(0), 0;
		cv::Mat tmp_inverse(3,4,CV_64FC1);
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 4; ++j) {
				tmp_inverse.at<double>(i, j) = proj_a(i, j);
			}
		}

		cv::Mat inverse_a;
		cv::invert(tmp_inverse, inverse_a, cv::DECOMP_SVD);
		Eigen::Matrix<double, 4, 3> inverse = Eigen::Matrix<double, 4, 3>::Zero();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 3; ++j) {
				inverse(i, j) = inverse_a.at<double>(i, j);
			}
		}

		Eigen::Matrix3d f_matrix = eInvSym * proj_b * inverse;
		//std::ofstream out(root_path_ + "/../txtFiles/f.txt", std::ios_base::out|std::ios_base::app);
		//out << f_matrix << std::endl << '\n';
		//out.close();
		return f_matrix;
	}

	/*	*a tool struct to store some infomation needed
	*/
	struct Pose2Param {
		std::string name;
		shared_ptr<View> view_ptr;
		shared_ptr<IntrinsicBase> cam_ptr;
		const Pose3 pose;
		Pose2Param() = default;
		Pose2Param(const string &s, shared_ptr<View> v, shared_ptr<IntrinsicBase> c, const Pose3 p) :name(s),view_ptr(v), cam_ptr(c), pose(p) {};
	};


	std::pair<openMVG::Vec2, openMVG::Vec2> TxMeasure::compute_corresponding_point(TxMeasure::Index left, TxMeasure::Index right,
		std::pair<cv::Point2d, cv::Point2d> left_point, std::pair<cv::Point2d, cv::Point2d> right_point,
		const map<Index, Pose2Param> &pre_info) {

		const auto &left_info = pre_info.at(left);	
		const auto &right_info = pre_info.at(right);
		const auto left_name = root_path_ + "/" + left_info.name;
		const auto right_name = root_path_ + "/" + right_info.name;
		//std::cout << left_info.name << ' ' << right_name << '\n';
		const auto &view1 = left_info.view_ptr;
		const auto &cam1 = left_info.cam_ptr;
		const auto &pose1 = left_info.pose;
		
		const auto &view2 = right_info.view_ptr;
		const auto &cam2 = right_info.cam_ptr;
		const auto &pose2 = right_info.pose;

		const auto F = compute_fundamental_matrix(left, right, view1, cam1, pose1, view2, cam2, pose2);

		/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
		Eigen::Vector3d two_epilines[2]{};
		openMVG::Vec2 t_1(left_point.first.x, left_point.first.y);
		//openMVG::Vec2 l_pt_1 = cam1->get_ud_pixel(t_1);
		openMVG::Vec2 l_pt_1 = t_1;
		openMVG::Vec3 right_line_1 = F*openMVG::Vec3(l_pt_1(0), l_pt_1(1), 1);
		two_epilines[0] = right_line_1;

		openMVG::Vec2 t_2(left_point.second.x, left_point.second.y);
		//openMVG::Vec2 l_pt_2 = cam1->get_ud_pixel(t_2);
		openMVG::Vec2 l_pt_2 = t_2;
		openMVG::Vec3 right_line_2 = F*openMVG::Vec3(l_pt_2(0), l_pt_2(1), 1);
		two_epilines[1] = right_line_2;

	
		openMVG::Vec2 r_tmp_a(right_point.first.x, right_point.first.y);
		//openMVG::Vec2 r_tmp_a1 = cam2->get_ud_pixel(r_tmp_a);
		openMVG::Vec2 r_tmp_a1 = r_tmp_a;
		auto x1 = r_tmp_a1(0);
		auto y1 = r_tmp_a1(1);
		openMVG::Vec2 r_tmp_b(right_point.second.x, right_point.second.y);
		//openMVG::Vec2 r_tmp_b1 = cam2->get_ud_pixel(r_tmp_b);
		openMVG::Vec2 r_tmp_b1 = r_tmp_b;
		auto x2 = r_tmp_b1(0);
		auto y2 = r_tmp_b1(1);
		Eigen::Vector3d point_line;

		auto a2 = y2 - y1, b2 = x1 - x2, c2 = x2*y1 - x1*y2;
		openMVG::Vec2 aaa[2];
		for (int i = 0; i < 2; ++i) {
			auto a1 = two_epilines[i](0);
			auto b1 = two_epilines[i](1);
			auto c1 = two_epilines[i](2);
			openMVG::Vec2 ans((b1*c2 - b2*c1) / (a1*b2 - a2*b1), (a2*c1 - a1*c2) / (a1*b2 - a2*b1));
			aaa[i] = ans;
		}

	/*	cv::Mat image_b = cv::imread(right_name);
		cv::Point2d start_r(aaa[0](0), aaa[0](1));
		cv::Point2d end_r(aaa[1](0), aaa[1](1));
		cv::line(image_b, start_r, end_r, cv::Scalar(255,255, 255), 1);
		string epipolar_path = root_path_ + "/../txtFiles/" + std::to_string(left) + std::to_string(right) + ".jpg";
		imwrite(epipolar_path, image_b);*/

		std::pair<openMVG::Vec2, openMVG::Vec2> result = make_pair(aaa[0], aaa[1]);
		return result;
	}


	vector<Eigen::Vector3d> TxMeasure::tranglation_2_view_rotation_matrix( map<Index, Pose2Param> &poseInfoForCorresponding) {
		auto line_num = lines_.size();
		vector<std::pair<cv::Point2d, cv::Point2d> >origin_point;
		origin_point.reserve(2);
		vector<std::pair<openMVG::Vec2, openMVG::Vec2> > corres_point;
		corres_point.reserve(2);
		vector<Index> view_id;
		view_id.reserve(2);
		vector<Eigen::Vector3d> real_point;
		real_point.reserve((line_num - 1)*line_num * 4);
		openMVG::Triangulation tri_1;
		openMVG::Triangulation tri_2;
		for (auto iter_i = lines_.cbegin(); iter_i != lines_.cend(); ++iter_i) {
			for (auto iter_j = lines_.cbegin(); iter_j != lines_.cend(); ++iter_j) {
				if (iter_i == iter_j)
					continue;
				origin_point.clear();
				corres_point.clear();
				view_id.clear();
				view_id.push_back(image2poseId_[iter_i->first]);
				view_id.push_back(image2poseId_[iter_j->first]);
				openMVG::Vec2 tmp_point_a(iter_i->second.first.x, iter_i->second.first.y);
				/*openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);*/
				openMVG::Vec2 tmp_point_b(iter_i->second.second.x, iter_i->second.second.y);
				/**********************************************/
			/*		double diff_y = std::abs(tmp_point_a(1) - tmp_point_b(1)) / 2;
					double line_A = tmp_point_b(1) - tmp_point_a(1);
					double line_B = tmp_point_a(0) - tmp_point_b(0);
					if (tmp_point_a(1) > tmp_point_b(1)) {
						tmp_point_a(0) -= line_B / line_A * diff_y;
						tmp_point_a(1) += diff_y;
						tmp_point_b(0) += line_B / line_A * diff_y;
						tmp_point_b(1) -= diff_y;
					}
					else {
						tmp_point_a(0) += line_B / line_A * diff_y;
						tmp_point_a(1) -= diff_y;
						tmp_point_b(0) -= line_B / line_A * diff_y;
						tmp_point_b(1) += diff_y;
					}*/
				/**********************************************/
				//openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);
				//openMVG::Vec2 tmp_point_b1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_b);
				openMVG::Vec2 tmp_point_a1 = tmp_point_a;
				openMVG::Vec2 tmp_point_b1 = tmp_point_b;
				corres_point.push_back({ tmp_point_a1,tmp_point_b1 });
				auto corr = compute_corresponding_point(view_id[0], view_id[1], iter_i->second, iter_j->second, poseInfoForCorresponding);
				corres_point.push_back({ corr.first,corr.second });
				tri_1.clear();
				tri_2.clear();
				for (unsigned i = 0; i < 2; ++i) {
					auto pose_nn = poseInfoForCorresponding[view_id[i]].pose;
					auto proj = poseInfoForCorresponding[view_id[i]].cam_ptr->get_projective_equivalent(pose_nn);
					//openMVG::Vec2 tmp_a = corres_point[i].first;
					//openMVG::Vec2 tmp_b = corres_point[i].second;
					//openMVG::Vec2 tmp_ab =  poseInfoForCorresponding[view_id[i]].cam_ptr->get_ud_pixel(tmp_a);
					//openMVG::Vec2 tmp_ba =  poseInfoForCorresponding[view_id[i]].cam_ptr->get_ud_pixel(tmp_b);
					tri_1.add(proj, corres_point[i].first);
					tri_2.add(proj, corres_point[i].second);
					//tri_1.add(proj, tmp_ab);
					//tri_2.add(proj, tmp_ba);
				}
				auto point_a = tri_1.compute();
				auto point_b = tri_2.compute();
				Eigen::Vector3d point_a1(point_a(0), point_a(1), point_a(2));
				Eigen::Vector3d point_b1(point_b(0), point_b(1), point_b(2));
				for (unsigned i = 0; i < 2; ++i) {
					Eigen::Vector3d point_a_R = poseInfoForCorresponding[view_id[i]].pose.rotation() * point_a1;
					Eigen::Vector3d point_a_T = point_a_R + translations_[view_id[i]];
					Eigen::Vector3d point_b_R = poseInfoForCorresponding[view_id[i]].pose.rotation() * point_b1;
					Eigen::Vector3d point_b_T = point_b_R + translations_[view_id[i]];
					Eigen::Vector3d real_point_a = V2R_rotations_[view_id[i]] * point_a_T;
					Eigen::Vector3d real_point_b = V2R_rotations_[view_id[i]] * point_b_T;
					real_point.push_back(real_point_a);
					real_point.push_back(real_point_b);
				}
			}
		}
		return real_point;
	}


	vector<Eigen::Vector3d> TxMeasure::tranglation_2_view_rotation_vector(map<Index, Pose2Param> &poseInfoForCorresponding) {
		auto line_num = lines_.size();
		vector<std::pair<cv::Point2d, cv::Point2d> >origin_point;
		origin_point.reserve(2);
		vector<std::pair<openMVG::Vec2, openMVG::Vec2> > corres_point;
		corres_point.reserve(2);
		vector<Index> view_id;
		view_id.reserve(2);
		vector<Eigen::Vector3d> real_point;
		real_point.reserve((line_num - 1)*line_num * 4);
		openMVG::Triangulation tri_1;
		openMVG::Triangulation tri_2;
		for (auto iter_i = lines_.cbegin(); iter_i != lines_.cend(); ++iter_i) {
			for (auto iter_j = lines_.cbegin(); iter_j != lines_.cend(); ++iter_j) {
				if (iter_i == iter_j)
					continue;
				origin_point.clear();
				corres_point.clear();
				view_id.clear();
				view_id.push_back(image2poseId_[iter_i->first]);
				view_id.push_back(image2poseId_[iter_j->first]);
				openMVG::Vec2 tmp_point_a(iter_i->second.first.x, iter_i->second.first.y);
				/*openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);*/
				openMVG::Vec2 tmp_point_b(iter_i->second.second.x, iter_i->second.second.y);
				/*****************************************************************************************/
				/*double diff_y = std::abs(tmp_point_a(1) - tmp_point_b(1)) / 2;
				double line_A = tmp_point_b(1) - tmp_point_a(1);
				double line_B = tmp_point_a(0) - tmp_point_b(0);
				if (tmp_point_a(1) > tmp_point_b(1)) {
					tmp_point_a(0) -= line_B / line_A * diff_y;
					tmp_point_a(1) += diff_y;
					tmp_point_b(0) += line_B / line_A * diff_y;
					tmp_point_b(1) -= diff_y;
				}
				else {
					tmp_point_a(0) += line_B / line_A * diff_y;
					tmp_point_a(1) -= diff_y;
					tmp_point_b(0) -= line_B / line_A * diff_y;
					tmp_point_b(1) += diff_y;
				}*/

				/****************************************************************************************/
				//openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);
				//openMVG::Vec2 tmp_point_b1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_b);
				openMVG::Vec2 tmp_point_a1 = tmp_point_a;
				openMVG::Vec2 tmp_point_b1 = tmp_point_b;
				corres_point.push_back({ tmp_point_a1,tmp_point_b1 });
				auto corr = compute_corresponding_point(view_id[0], view_id[1], iter_i->second, iter_j->second, poseInfoForCorresponding);
				corres_point.push_back({ corr.first,corr.second });
				tri_1.clear();
				tri_2.clear();
				for (unsigned i = 0; i < 2; ++i) {
					auto pose_nn = poseInfoForCorresponding[view_id[i]].pose;
					auto proj = poseInfoForCorresponding[view_id[i]].cam_ptr->get_projective_equivalent(pose_nn);
					tri_1.add(proj, corres_point[i].first);
					tri_2.add(proj, corres_point[i].second);
				}
				auto point_a = tri_1.compute();
				auto point_b = tri_2.compute();
				Eigen::Vector3d point_a1(point_a(0), point_a(1), point_a(2));
				Eigen::Vector3d point_b1(point_b(0), point_b(1), point_b(2));
				for (unsigned i = 0; i < 2; ++i) {
					Eigen::Vector3d point_a_R = poseInfoForCorresponding[view_id[i]].pose.rotation() * point_a1;
					Eigen::Vector3d point_a_T = point_a_R + translations_[view_id[i]];
					Eigen::Vector3d point_b_R = poseInfoForCorresponding[view_id[i]].pose.rotation() * point_b1;
					Eigen::Vector3d point_b_T = point_b_R + translations_[view_id[i]];
					Eigen::Vector3d real_point_a = V2R_rotation_vector_[view_id[i]] * point_a_T;
					Eigen::Vector3d real_point_b = V2R_rotation_vector_[view_id[i]] * point_b_T;
					real_point.push_back(real_point_a);
					real_point.push_back(real_point_b);
				}
			}
		}
		return real_point;
	}

	vector<Eigen::Vector3d> TxMeasure::tranglation_all_view_vector_matrix_MI(map<Index, Pose2Param> &poseInfoForCorresponding) {
		auto line_num = lines_.size();
		vector<Index> ids;
		ids.reserve(line_num);
		for (const auto &k : lines_) {
			ids.push_back(image2poseId_[k.first]);
		}

		vector<Index> view_id(2, -1);
		vector<Eigen::Vector3d> real_point;
		real_point.reserve((line_num - 1)*line_num * 4);
		openMVG::Triangulation tri_1;
		openMVG::Triangulation tri_2;

		for (auto iter_i = lines_.cbegin(); iter_i != lines_.cend(); ++iter_i) {
			tri_1.clear();
			tri_2.clear();

			view_id[0] = image2poseId_[iter_i->first];
			openMVG::Vec2 tmp_point_a(iter_i->second.first.x, iter_i->second.first.y);
			openMVG::Vec2 tmp_point_b(iter_i->second.second.x, iter_i->second.second.y);
			//openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);
			//openMVG::Vec2 tmp_point_b1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_b);
			openMVG::Vec2 tmp_point_a1 = tmp_point_a;
			openMVG::Vec2 tmp_point_b1 = tmp_point_b;
			auto pose_mark = poseInfoForCorresponding[view_id[0]].pose;
			auto proj_mark = poseInfoForCorresponding[view_id[0]].cam_ptr->get_projective_equivalent(pose_mark);
			tri_1.add(proj_mark, tmp_point_a1);
			tri_2.add(proj_mark, tmp_point_b1);

			for (auto iter_j = lines_.cbegin(); iter_j != lines_.cend(); ++iter_j) {
				if (iter_i == iter_j)
					continue;
				view_id[1] = image2poseId_[iter_j->first];
				auto corr = compute_corresponding_point(view_id[0], view_id[1], iter_i->second, iter_j->second, poseInfoForCorresponding);
				//openMVG::Vec2 tmp_a = poseInfoForCorresponding[view_id[1]].cam_ptr->get_ud_pixel(corr.first);
				//openMVG::Vec2 tmp_b = poseInfoForCorresponding[view_id[1]].cam_ptr->get_ud_pixel(corr.second);
				auto pose_nn = poseInfoForCorresponding[view_id[1]].pose;
				auto proj_nn = poseInfoForCorresponding[view_id[1]].cam_ptr->get_projective_equivalent(pose_nn);
				tri_1.add(proj_nn, corr.first);
				tri_2.add(proj_nn, corr.second);
			}
			auto point_a = tri_1.compute();
			auto point_b = tri_2.compute();
			Eigen::Vector3d point_a1(point_a(0), point_a(1), point_a(2));
			Eigen::Vector3d point_b1(point_b(0), point_b(1), point_b(2));
			for (unsigned i = 0; i < line_num; ++i) {
				Eigen::Vector3d point_a_R = poseInfoForCorresponding[ids[i]].pose.rotation() * point_a1;
				Eigen::Vector3d point_a_T = point_a_R + translations_[ids[i]];
				Eigen::Vector3d point_b_R = poseInfoForCorresponding[ids[i]].pose.rotation() * point_b1;
				Eigen::Vector3d point_b_T = point_b_R + translations_[ids[i]];
				Eigen::Vector3d real_point_a = V2R_rotations_[ids[i]] * point_a_T;
				Eigen::Vector3d real_point_b = V2R_rotations_[ids[i]] * point_b_T;
				real_point.push_back(real_point_a);
				real_point.push_back(real_point_b);
			}

		}
		return real_point;
	}

	vector<Eigen::Vector3d> TxMeasure::tranglation_all_view_vector_matrix(map<Index, Pose2Param> &poseInfoForCorresponding) {
		auto line_num = lines_.size();
		vector<Index> ids;
		ids.reserve(line_num);
		for (const auto &k : lines_) {
			ids.push_back(image2poseId_[k.first]);
		}

		vector<Index> view_id(2, -1);
		vector<Eigen::Vector3d> real_point;
		real_point.reserve((line_num - 1)*line_num * 4);
		openMVG::Triangulation tri_1;
		openMVG::Triangulation tri_2;

		for (auto iter_i = lines_.cbegin(); iter_i != lines_.cend(); ++iter_i) {
			tri_1.clear();
			tri_2.clear();

			view_id[0] = image2poseId_[iter_i->first];
			openMVG::Vec2 tmp_point_a(iter_i->second.first.x, iter_i->second.first.y);
			openMVG::Vec2 tmp_point_b(iter_i->second.second.x, iter_i->second.second.y);
			//openMVG::Vec2 tmp_point_a1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_a);
			//openMVG::Vec2 tmp_point_b1 = poseInfoForCorresponding[view_id[0]].cam_ptr->get_ud_pixel(tmp_point_b);
			openMVG::Vec2 tmp_point_a1 = tmp_point_a;
			openMVG::Vec2 tmp_point_b1 = tmp_point_b;
			auto pose_mark = poseInfoForCorresponding[view_id[0]].pose;
			auto proj_mark = poseInfoForCorresponding[view_id[0]].cam_ptr->get_projective_equivalent(pose_mark);
			tri_1.add(proj_mark, tmp_point_a1);
			tri_2.add(proj_mark, tmp_point_b1);

			for (auto iter_j = lines_.cbegin(); iter_j != lines_.cend(); ++iter_j) {
				if (iter_i == iter_j)
					continue;
				view_id[1] = image2poseId_[iter_j->first];
				auto corr = compute_corresponding_point(view_id[0], view_id[1], iter_i->second, iter_j->second, poseInfoForCorresponding);
				//openMVG::Vec2 tmp_a = poseInfoForCorresponding[view_id[1]].cam_ptr->get_ud_pixel(corr.first);
				//openMVG::Vec2 tmp_b = poseInfoForCorresponding[view_id[1]].cam_ptr->get_ud_pixel(corr.second);
				auto pose_nn = poseInfoForCorresponding[view_id[1]].pose;
				auto proj_nn = poseInfoForCorresponding[view_id[1]].cam_ptr->get_projective_equivalent(pose_nn);
				tri_1.add(proj_nn, corr.first);
				tri_2.add(proj_nn, corr.second);
			}
			auto point_a = tri_1.compute();
			auto point_b = tri_2.compute();
			Eigen::Vector3d point_a1(point_a(0), point_a(1), point_a(2));
			Eigen::Vector3d point_b1(point_b(0), point_b(1), point_b(2));
			for (unsigned i = 0; i < line_num; ++i) {
				Eigen::Vector3d point_a_R = poseInfoForCorresponding[ids[i]].pose.rotation() * point_a1;
				Eigen::Vector3d point_a_T = point_a_R;// +translations_[ids[i]];
				Eigen::Vector3d point_b_R = poseInfoForCorresponding[ids[i]].pose.rotation() * point_b1;
				Eigen::Vector3d point_b_T = point_b_R;// +translations_[ids[i]];
				Eigen::Vector3d real_point_a = V2R_rotation_vector_[ids[i]] * point_a_T;
				Eigen::Vector3d real_point_b = V2R_rotation_vector_[ids[i]] * point_b_T;
				real_point.push_back(real_point_a);
				real_point.push_back(real_point_b);
			}
		}
		return real_point;
	}


	void TxMeasure::my_go(unsigned m_num_view) {

		using c_iterator = map<string, pair<cv::Point2d, cv::Point2d> >::const_iterator;
		c_iterator iter_end = lines_.end();
		unsigned size = static_cast<unsigned>(lines_.size());
		map<Index, Pose2Param> poseInfoForCorresponding;

		for (const auto &k : lines_) {
			Index id = image2poseId_[k.first];
			string name = k.first;
			auto &view = my_sfm_data_.views[id];
			auto &cam = my_sfm_data_.GetIntrinsics().at(view->id_intrinsic);
			const auto pose = my_sfm_data_.GetPoseOrDie(view.get());
			poseInfoForCorresponding.insert({ id,Pose2Param(name,view, cam, pose) });
		}


		auto real_point_by_read_rotation = tranglation_2_view_rotation_matrix(poseInfoForCorresponding);
		auto real_point_by_vector_rotation = tranglation_2_view_rotation_vector(poseInfoForCorresponding);
		auto real_point_all_vector = tranglation_all_view_vector_matrix(poseInfoForCorresponding);
		auto real_point_all_rotation = tranglation_all_view_vector_matrix_MI(poseInfoForCorresponding);

		double anr_1 = ComputeFuyang(real_point_by_read_rotation);
		double anr_2 = AvgVector(real_point_by_read_rotation);
		double anr_t = ComputeShuiPing_m(real_point_by_read_rotation, "r.txt",anr_2);

		double anv_1 = ComputeFuyang(real_point_by_vector_rotation);
		double anv_2 = AvgVector(real_point_by_vector_rotation);
		double anv_t = ComputeShuiPing_m(real_point_by_vector_rotation, "v.txt",anv_2);

		double anar_1 = ComputeFuyang(real_point_all_rotation);
		double anar_2 = AvgVector(real_point_all_rotation);
		double anar_t = ComputeShuiPing_m(real_point_all_rotation, "ra.txt",anar_2);

		double anav_1 = ComputeFuyang(real_point_all_vector);
		double anav_2 = AvgVector(real_point_all_vector);
		double anav_y = ComputeShuiPing_m(real_point_all_vector, "va.txt",anav_2);

		std::ofstream out(root_path_ + "/../R.txt");
		out << "anr_1: " << anr_1 << "\nanr_2: " << anr_2
			<< "\nanv_1: " << anv_1 << "\nanv_2: " << anv_2
			<< "\nanar_1: " << anar_1 << "\nanar_2: " << anar_2
			<< "\nanav_1: " << anav_1 << "\nanav_2: " << anav_2;
		out.close();
	
	}	
	


	Eigen::Vector2d GetVector(double tx, double ty, double tz, double rx, double ry, double rz) {
		/*	consider the point with smaller z value as origin point
		!!!actually not proper, because only a line can not represent the orientation of a plane!!!
		*/
		double x, y;
		if (tz > rz) {
			x = tx - rx;
			y = ty - ry;
		}
		else {
			x = rx - tx;
			y = ry - ty;
		}
		Eigen::Vector2d re(x,y);
		return re;
	}


	void NormalizeVector(Eigen::Vector2d &v) {
		double a = v(0);
		double b = v(1);
		v(0) = a / std::sqrt(a*a + b*b);
		v(1) = b / std::sqrt(a*a + b*b);

	}
	double TxMeasure::AvgVector(const vector<Eigen::Vector3d> &points) {
		vector<Eigen::Vector2d> orient_vector;
		orient_vector.reserve((unsigned)points.size() / 2);
		for (unsigned i = 0; i < points.size(); i += 2) {
			Eigen::Vector2d an = GetVector(points[i](0), points[i](1), points[i](2),
				points[i + 1](0), points[i + 1](1), points[i + 1](2));
			orient_vector.push_back(an);
		}

		for (auto &k : orient_vector) {
			NormalizeVector(k);
		}
		double one = 0;
		double two = 0;
		for (const auto&k : orient_vector) {
			one += k(0);
			two += k(1);
		}

		double angle = two / std::sqrt(one*one + two*two);
		angle = acos(angle) * 180 / M_PI;
		if (one < 0) {
			angle = 360 - angle;
		}
		return angle;
	}

}
