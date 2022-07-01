#pragma once


// Open3D
#include "open3d/Open3D.h"




namespace Cockroach
{


	inline Eigen::Matrix4d plane_to_plane(
		Eigen::Vector3d O0, Eigen::Vector3d X0, Eigen::Vector3d Y0, Eigen::Vector3d Z0,
		Eigen::Vector3d O1, Eigen::Vector3d X1, Eigen::Vector3d Y1, Eigen::Vector3d Z1

	) {
		Eigen::Vector3d T0(0 - O0.x(), 0 - O0.y(), 0 - O0.z());
		//Move to origin -> T0 translates point P0 to (0,0,0)

		//Rotate A ->
		Eigen::Matrix3d F0;
		F0 << X0.x(), X0.y(), X0.z(),
			Y0.x(), Y0.y(), Y0.z(),
			Z0.x(), Z0.y(), Z0.z();

		//Rotate B ->
		Eigen::Matrix3d F1;
		F1 << X1.x(), Y1.x(), Z1.x(),
			X1.y(), Y1.y(), Z1.y(),
			X1.z(), Y1.z(), Z1.z();

		//Move to 3d -> T1 translates (0,0,0) to point P1
		Eigen::Vector3d T1(O1.x() - 0, O1.y() - 0, O1.z() - 0);


		Eigen::Matrix3d F = F1 * F0;



		//Combine translation and rotation
		Eigen::Matrix4d A;
		A.setIdentity();
		//A.block<3, 3>(0, 0) = F0;
		A.block<3, 1>(0, 3) = T0;

		Eigen::Matrix4d B;
		B.setIdentity();
		B.block<3, 3>(0, 0) = F;
		//B.block<3, 1>(0, 3) = T1;

		Eigen::Matrix4d C;
		C.setIdentity();
		//C.block<3, 3>(0, 0) = F1;
		C.block<3, 1>(0, 3) = T1;



		//Output
		return C * B * A;
	}

	inline void orient_pointcloud(
		std::shared_ptr< open3d::geometry::PointCloud >& cloud,
		Eigen::Vector3d& center0, Eigen::Vector3d& XAxis0, Eigen::Vector3d& YAxis0, Eigen::Vector3d& ZAxis0,
		Eigen::Vector3d& center1, Eigen::Vector3d& XAxis1, Eigen::Vector3d& YAxis1, Eigen::Vector3d& ZAxis1
	) {
		Eigen::Matrix4d transformation_matrix = plane_to_plane(
			center0, XAxis0, YAxis0, ZAxis0,
			center1, XAxis1, YAxis1, ZAxis1);

		*cloud = cloud->Transform(transformation_matrix);
	}

}

