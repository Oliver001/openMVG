#include <direct.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Eigen\Eigen\Dense"
#include "tx_measure.h"
#include "myIOutility.h"

namespace TX {
	/*	create directories will be need in the next process.
	*/
	void MkDirectory(const std::string base_name) {
		mkdir((base_name + "/../" + "txtFiles").c_str());
		mkdir((base_name + "/../" + "points3D").c_str());
		mkdir((base_name + "/../" + "epipolarImg").c_str());
		mkdir((base_name + "/../" + "GPS").c_str());
	}

	/*	find the location of the parameters in the txt file
		!just read them line by line
	*/
	void LocateSensorParamter(std::ifstream &in, const std::string &type, std::vector<double> &out_parameter) {
		std::string parameter_name;
		while (in >> parameter_name) {
			if (parameter_name == "GPS:")
				break;
		}
		for (unsigned i = 0; i < 3; ++i) {
			double t;
			in >> t;
			out_parameter[i] = t;
		}

		while (in >> parameter_name) {
			if (parameter_name == type)
				break;
		}
		for (unsigned i = 3; i < 12; ++i) {
			double t;
			in >> t;
			out_parameter[i] = t;
		}
		return;
	}


	void LocateAngle(std::ifstream &in, std::vector<double> &param) {
		std::string parameter_name;
		while (in >> parameter_name) {
			if (parameter_name == "GPS:")
				break;
		}
		for (unsigned i = 0; i < 3; ++i) {
			double t;
			in >> t;
			param[i] = t;
		}
		while (in >> parameter_name) {
			if (parameter_name == "Orientation_NEW_API:")
				break;
		}
		for (unsigned i = 3; i < 6; ++i) {
			double t;
			in >> t;
			param[i] = t;
		}
		while (in >> parameter_name) {
			if (parameter_name == "OrientationFromVector:")
				break;
		}
		for (unsigned i = 6; i < 9; ++i) {
			double t;
			in >> t;
			param[i] = t;
		}
		return;
	}

	/*	read parameters got by the phone sensors from the txt files corresponding with the images
	*/
	bool ReadSensorTxt(const std::string filename, std::vector<double> &out_parameter, SensorType flag) {

		std::ifstream fin(filename);
		if (!fin.is_open()) {
			return false;
		}

		std::string flag_name;
		switch (flag)
		{
		case SensorType::Orientation:
			flag_name = "RotationMatrix:";
			LocateSensorParamter(fin, flag_name, out_parameter);
			break;
		case SensorType::RemappedRotation:
			//flag_name = "RemappedRotationMatrixFromVector:";
			flag_name = "RemappedRotationMatrix:";
			LocateSensorParamter(fin, flag_name, out_parameter);
			break;
		case SensorType::RotationMatrixFromVector:
			flag_name = "RotationMatrixFromVector:";
			LocateSensorParamter(fin, flag_name, out_parameter);
			break;
		case SensorType::angle:
			LocateAngle(fin, out_parameter);
			break;
		default:
			break;
		}

		fin.close();
		return true;
	}

	/*	print the class member we got for test aim
		@ type is the member name wanted to be show
	*/
	void f_DisplayInfomation(const TxMeasure &tx, DisplayType type) {
		if (type == DisplayType::input_file_name) {
			std::cout << "sfm_data.json path: " << tx.input_file_name_ << '\n';
			return;
		}
		if (type == DisplayType::root_path) {
			std::cout << "image directory: " << tx.root_path_ << '\n';
		}

		if (type == DisplayType::V2R_rotations) {
			std::cout << "virtual to reality rotation matrices and its corresponding pose id:\n";
			for (auto &k : tx.V2R_rotations_) {
				std::cout << k.first << '\n' << k.second << '\n';
			}
		}

		if (type == DisplayType::GPS) {
			std::cout << "GPS infomation and its corresponding pose id:\n";
			for (auto &k : tx.GPS_) {
				std::cout << k.first << '\n';
				for (auto &g : k.second) {
					std::cout << g << '\t';
				}
				std::cout << '\n';
			}
		}
		if (type == DisplayType::image2poseId) {
			std::cout << "image names and their corredponding id:\n";
			for (auto &k : tx.image2poseId_) {
				std::cout << k.first << '\t' << k.second << '\n';
			}
		}
	}
}