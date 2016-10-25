#pragma once
#ifndef DECREPIT_H_
#define DECREPIT_H_
#include <vector>
#include <map>
#include <string>
#include <opencv2\line_descriptor.hpp>

using std::vector;
using std::map;
using std::string;
namespace TX {
	namespace Capture {
		map<string, std::pair<cv::Point2d, cv::Point2d> > *d_CaptureLine(const string &sfm_data_json);
		map<string, std::pair<cv::Point2d, cv::Point2d> > * ReadLineFromFile(const string &line_file);
		void ShowReadLine(const string &line_image_path, const map<string, std::pair<cv::Point2d, cv::Point2d> > *ptr);
	}
}
#endif // !DECREPIT_H_
