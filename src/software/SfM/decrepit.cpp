/*	*this file is about cutting subimage from the origin image 
	and detect line from the subimage
	!!!CAN NOT BE USED IN PHONE CLIENT
*/
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <utility>
#include <iomanip>
#include <algorithm>
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\line_descriptor.hpp"
#include "openMVG\sfm\sfm_data.hpp"
#include "openMVG\sfm\sfm_data_io.hpp"
#include "decrepit.h"

using std::map;
using std::vector;
using std::string;
using cv::Mat;
using cv::imwrite;
using cv::imread;
using cv::imshow;
using cv::namedWindow;
using cv::Point;
using cv::Point2d;
using cv::Rect;
using cv::Scalar;
using cv::line_descriptor::KeyLine;
using namespace openMVG::sfm;
namespace TX {
	namespace Capture {
		struct MouseData {
			int x = 0;
			int y = 0;
			int x_end = 0;
			int y_end = 0;
			int width = 0;
			int height;
			bool flag = false;
			Mat image;
			string win_name;
		};
		std::ostream &operator<<(std::ostream &os, const MouseData &out_data) {
			os << "start: " << out_data.x << ' ' << out_data.y << '\n';
			os << "end: " << out_data.x_end << ' ' << out_data.y_end << '\n';
			os << "size: " << out_data.width << ' ' << out_data.height << '\n';
			return os;
		}

		/*
			setMouseCallBack use this callback function
		*/
		void OnMouse(int event, int x, int y, int flags, void *origin_mouse_data) {
			MouseData *m_origin_mouse_data = static_cast<MouseData *>(origin_mouse_data);
			if (m_origin_mouse_data->flag == false) {
				if (event == cv::EVENT_LBUTTONDOWN) {
					m_origin_mouse_data->flag = true;
					m_origin_mouse_data->x = x;
					m_origin_mouse_data->y = y;
					Mat show_image = m_origin_mouse_data->image;
					cv::circle(show_image, Point(x, y), 2, Scalar(0, 0, 255));
					const string text = "(" + std::to_string(x) + "," + std::to_string(y) + ")";
					cv::putText(show_image, text, Point(x, y), cv::FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255), 3);
					imshow(m_origin_mouse_data->win_name, show_image);
				}
			}
			else if (event == cv::EVENT_LBUTTONDOWN & cv::EVENT_MOUSEMOVE) {
				m_origin_mouse_data->x_end = x;
				m_origin_mouse_data->y_end = y;
			}
			else if (event == cv::EVENT_LBUTTONUP) {
				m_origin_mouse_data->x_end = std::min(m_origin_mouse_data->image.size().width, x);
				m_origin_mouse_data->y_end = std::min(m_origin_mouse_data->image.size().height, y);

				cv::circle(m_origin_mouse_data->image, Point(m_origin_mouse_data->x_end, m_origin_mouse_data->y_end),
					2, Scalar(255, 0, 0));

				cv::putText(m_origin_mouse_data->image,
					std::to_string(m_origin_mouse_data->x_end) + " " + std::to_string(m_origin_mouse_data->y_end),
					Point(m_origin_mouse_data->x_end + 3, m_origin_mouse_data->y_end + 3),
					cv::FONT_HERSHEY_COMPLEX, 2, Scalar(0, 255, 0), 3);

				m_origin_mouse_data->width = m_origin_mouse_data->x_end - m_origin_mouse_data->x;
				m_origin_mouse_data->height = m_origin_mouse_data->y_end - m_origin_mouse_data->y;

				cv::rectangle(m_origin_mouse_data->image, Point(m_origin_mouse_data->x, m_origin_mouse_data->y),
					Point(m_origin_mouse_data->x_end, m_origin_mouse_data->y_end), Scalar(0, 255, 0), 3);

				cv::putText(m_origin_mouse_data->image,
					std::to_string(m_origin_mouse_data->width) + " " + std::to_string(m_origin_mouse_data->height),
					Point(m_origin_mouse_data->x + 100, m_origin_mouse_data->y + 50),
					cv::FONT_HERSHEY_COMPLEX, 2, Scalar(0, 255, 0), 3);
				imshow(m_origin_mouse_data->win_name, m_origin_mouse_data->image);
				m_origin_mouse_data->flag = false;
			}
		}



		// *order: red, blue,green, black, white 
		Scalar color_type[] = { Scalar(0, 0, 255),Scalar(0, 255, 0), Scalar(255, 0, 0),
			   Scalar(255, 255, 255),Scalar(0, 0, 0) };

		/*	*detect lines from the subimage, and choose line manually.
			@param image_names: small image_names, full path
			@param origin_names: original image_names, olny name no directory path
			@param ptr: output, original image_name and its chosen line.
		*/
		void d_DetectLineFromSubimage(const vector<string> &image_names, const vector<std::pair<string,Rect> > &origin_names,
			const string &root_path, map<string, KeyLine> *ptr) {
			cv::line_descriptor::BinaryDescriptor a;
			auto sort_criteria_greater = [](const KeyLine &first, const KeyLine &second) { return first.lineLength > second.lineLength; };
			vector<vector<KeyLine> > line_in_all_image;
			unsigned ind = 0;
			line_in_all_image.reserve(image_names.size());


			/*	read in the small images and detect line 
				choose first 5 longest line and  draw them on the small pictures
				write pictures with lines in files "_Ls.jpg"
			*/
			for (auto &k : image_names) {
				Mat sub_image = imread(k);
				line_in_all_image.push_back(vector<KeyLine>());
				vector<KeyLine> &keylines = line_in_all_image.back();
				a.detect(sub_image, keylines);
				std::sort(keylines.begin(), keylines.end(), sort_criteria_greater);
				unsigned draw_line_number = keylines.size() > 5 ? 5 : keylines.size();
				auto end_iter = keylines.cbegin();
				std::advance(end_iter, draw_line_number);
				unsigned color_i = 0;
				for (auto iter = keylines.cbegin(); iter != end_iter; ++iter) {
					auto start_point = Point(iter->startPointX, iter->startPointY);
					auto end_point = Point(iter->endPointX, iter->endPointY);
					auto mid_point = (start_point + end_point) / 2;
					cv::line(sub_image, start_point, end_point, color_type[color_i],1);
					cv::putText(sub_image, std::to_string(color_i), mid_point, cv::FONT_HERSHEY_COMPLEX, 1, color_type[color_i]);
					++color_i;
				}

				/*	!CREATE WINDOW HERE
				*/
				string window_name = std::to_string(ind) + " " + origin_names[ind].first;
				++ind;
				namedWindow(window_name, cv::WINDOW_AUTOSIZE);
				imshow(window_name, sub_image);
				cv::waitKey();
				imwrite(k + "_Ls.jpg", sub_image);
			}

			for (unsigned i = 0; i < image_names.size(); ++i) {
				int l = -1;
				std::cin >> l;
				if(l >= 0)
					ptr->operator[](origin_names[i].first) = line_in_all_image[i][l];
			}
			cv::destroyAllWindows();
			ind = 0;

			for (const auto &k : *ptr) {
				const string full_origin_name = root_path + "/../epipolarImg/s" + k.first;
				Mat image = imread(full_origin_name);
				const string image_target_line = root_path +"/../epipolarImg/s"+ k.first+"_L.jpg";
				Point start_point(k.second.startPointX, k.second.startPointY);
				Point end_point(k.second.endPointX, k.second.endPointY);
				cv::line(image, start_point, end_point, color_type[0]);
				string window_name = std::to_string(ind++) + " " + k.first;
				namedWindow(window_name, cv::WINDOW_AUTOSIZE);
				imshow(window_name, image);
				cv::waitKey();
				imwrite(image_target_line, image);
			}
			cv::destroyAllWindows();
		}

		/*
		*/
		struct CaptureUsing{
			//	*origin images directory
			string root_path_;	
			//	*origin images width and height
			unsigned width;
			unsigned height;
			//	*origin images' name and their pose id;
			map<string, unsigned> image2poseId_;
		};

		/*	initialize CaptureUsing struct
			because the struct stores the information about origin images
			we need this struct to find them.
		*/
		void d_Init_Capture_Struct(const string &path, CaptureUsing &cap) {
			SfM_Data m_sfm_data;
			Load(m_sfm_data, path, ESfM_Data(VIEWS | EXTRINSICS));
			const Poses &pose = m_sfm_data.poses;
			const Views &view = m_sfm_data.views;
			cap.root_path_ = m_sfm_data.s_root_path;
			for (auto it = pose.cbegin(); it != pose.cend(); ++it) {
				if (it->first != view.at(it->first)->id_pose)
					continue;
				const string image_name = view.at(it->first)->s_Img_path;
				cap.image2poseId_.insert({ image_name.substr(1),it->first });
				cap.height = view.at(it->first)->ui_height;
				cap.width = view.at(it->first)->ui_width;
			}
		}


		void DrawLineInOriginImage(const map<string, std::pair<Point, Point> > *ptr, const string &root_path) {
			for (const auto &k : *ptr) {
				const std::string origin_name = root_path + "/" + k.first;
				Mat image = imread(origin_name);
				cv::line(image, k.second.first, k.second.second, color_type[0]);
				imwrite(root_path + "/../epipolarImg/L_" + k.first, image);
			}
		}

		/*	*cut subimage and detect line from the subimage.
		write subimage into the epolarimage directory.
		return the map whose keys are parent image names and values are lines chosen manually
		*/
		map<string, std::pair<Point2d,Point2d> > *d_CaptureLine(const string &sfm_data_json_path) {
			CaptureUsing tx;
			d_Init_Capture_Struct(sfm_data_json_path, tx);
			vector<string> s_image_names;
			vector<std::pair<string,Rect> > origin_image_names;
			origin_image_names.reserve(tx.image2poseId_.size());

			namedWindow("origin_image", cv::WINDOW_NORMAL);
			for (const auto &k : tx.image2poseId_) {
				const string image_name = tx.root_path_ + "/" + k.first;
				Mat image = imread(image_name);
				MouseData for_mouse_callback;
				for_mouse_callback.image = image.clone();
				for_mouse_callback.win_name = "origin_image";
				
				cv::setMouseCallback("origin_image", OnMouse, &for_mouse_callback);
				imshow("origin_image", image);
				cv::waitKey(0);
				const string ROI_image_name = tx.root_path_ + "/../epipolarImg/s" + k.first;

				/*	store subimage's parent image name ,and subimage's position in parent image
					in the vector origin_image_names.
					the vector will be used in the future.
				*/
				Rect s_rectangle(for_mouse_callback.x, for_mouse_callback.y, for_mouse_callback.width, for_mouse_callback.height);
				origin_image_names.push_back({ k.first,s_rectangle });

				//	*cut sub image from parent image and  write them into epipolarImg directory.
				Mat ROI_image = image(s_rectangle);
				imwrite(ROI_image_name, ROI_image);
				s_image_names.push_back(ROI_image_name);	
				std::ofstream out(ROI_image_name + ".txt");
				out << for_mouse_callback;
				out.close();
			}
			cv::destroyAllWindows();
			map < string, KeyLine> * choose_line_ptr_d = new map<string, KeyLine>();
			d_DetectLineFromSubimage(s_image_names, origin_image_names, tx.root_path_, choose_line_ptr_d);
			map <string, std::pair<Point2d, Point2d> > *result_ptr_d = new map<string, std::pair<Point2d, Point2d> >();
			for (const auto &k : *choose_line_ptr_d) {
				auto iter = std::find_if(origin_image_names.cbegin(), origin_image_names.cend(), [&k](const auto &a) { return a.first == k.first; });
				cv::Point2d start(iter->second.x + k.second.startPointX, iter->second.y + k.second.startPointY);
				cv::Point2d end(iter->second.x + k.second.endPointX, iter->second.y + k.second.endPointY);
				std::pair<cv::Point2d, cv::Point2d> line(start, end);
				result_ptr_d->insert({ k.first,line });
			}
			delete choose_line_ptr_d;

			//DrawLineInOriginImage(result_ptr_d, tx.root_path_);

			std::ofstream out(tx.root_path_ + "/../line.txt");
			for (const auto&k : *result_ptr_d) {
				out << k.first << std::setprecision(15) << ' ' << k.second.first.x <<' '<< k.second.first.y
					<< ' ' << k.second.second.x << ' '<< k.second.second.y << std::endl;
			}
			out.close();
			return result_ptr_d;
		}


		map<string, std::pair<Point2d, Point2d> > * ReadLineFromFile(const string &line_file) {
			std::ifstream in(line_file);
			std::string one_line;
			map<string, std::pair<Point2d, Point2d> > *ptr = new map<string, std::pair<Point2d, Point2d> >();
			while (std::getline(in, one_line)) {
				std::istringstream istream(one_line);
				std::string pic_name;
				double one_1, one_2, two_1, two_2;
				istream >> pic_name >> one_1 >> one_2 >> two_1 >> two_2;
				Point2d one(one_1, one_2), two(two_1, two_2);
				ptr->insert({ pic_name,std::make_pair(one,two) });
			}
			return ptr;
		}

		void ShowReadLine(const string &line_image_path, const map<string,std::pair<Point2d,Point2d> > *ptr) {
			namedWindow("ReadLine");
			for (const auto&k : *ptr) {
				Mat image = imread(line_image_path + k.first);
				std::ifstream in(line_image_path + k.first + ".txt");
				string start;
				in >> start;
				int left_up_1, left_up_2;
				if (start == "start:") {
					in >> left_up_1; 
					in >> left_up_2;
				}
				cv::Point corner(left_up_1, left_up_2);
				cv::Point first(k.second.first.x - left_up_1, k.second.first.y - left_up_2);
				cv::Point second(k.second.second.x - left_up_1, k.second.second.y - left_up_2);
				cv::line(image, first, second, color_type[0]);
				imshow("ReadLine", image);
				cv::waitKey(0);
			}

			return;
		}

	}
}