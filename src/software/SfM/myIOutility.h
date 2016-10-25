#pragma once
#ifndef MY_IO_UTILITY_H_
#define MY_IO_UTILITY_H_
#include <string>
#include "Eigen\Eigen\Dense"
#include "tx_measure.h"
namespace TX {
	enum class SensorType { Orientation, RemappedRotation, RotationMatrix, RotationMatrixFromVector,angle };
	enum class DisplayType {
		input_file_name, root_path, lineId2poseId,
		image2poseId, V2R_rotations, lines,
		virtual_points, reality_points, GPS
	};

	void MkDirectory(const std::string base_name);

	bool ReadSensorTxt(const std::string filename, std::vector<double> &out_parameter, SensorType flag);

	void f_DisplayInfomation(const TxMeasure &tx, DisplayType type);
#endif // !MY_IO_UTILITY_H_
}