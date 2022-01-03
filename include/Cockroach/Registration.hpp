// #pragma once

// #include <iostream>

// // Open3D
// #include "open3d/Open3D.h"
// #include <open3d/pipelines/registration/RobustKernel.h>
// #include <open3d/pipelines/registration/ColoredICP.h>
// #include <open3d/pipelines/registration/FastGlobalRegistration.h>

// // Cilantro
// #include <cilantro/utilities/point_cloud.hpp>

// typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
// typedef std::shared_ptr<PC> PC_ptr;
// typedef open3d::pipelines::registration::RegistrationResult RS;


// namespace Cockroach 
// {
//     /// \summary Evaluate a possible first registration, it is mostly needed for a ICP.
//     /// \param threshold It's the maximal distance between two pair of points to be registered.
//     RS evaluateRegistration(std::shared_ptr<PC> cloudSource,
//                             std::shared_ptr<PC> cloudTarget,
//                             float threshold)
//     {
//         // Call the open3d method
//         auto registration_result = open3d::pipelines::registration::EvaluateRegistration(*cloudSource, *cloudTarget, threshold);
//         return registration_result;
//     };

//     /// \summary ICP point-to-point do not need normals but it's more expensive and prone to shifting on the same plane
//     /// \param initTrans The initial transformation to refine.
//     /// \param scaleIt Scale of not the source cloud to the target one.
//     /// \param iterations Maximal number of iterations.
//     /// \param rel_fitness Boundary of fitness (the amount of corresponding points).
//     /// \param rel_RMSE Boundary of RMSE (the precision of the registration).
//     RS ICPPointToPoint(std::shared_ptr<PC> cloudSource,
//                     std::shared_ptr<PC> cloudTarget,
//                     Eigen::Matrix4d_u initTrans,
//                     float threshold,
//                     bool scaleIt = false,
//                     int iterations = 30,
//                     double rel_fitness = 1E-06,
//                     double rel_RMSE = 1E-06)
//     {
//         // Call Open3D method
//         auto reg_result_ICP_PTP = open3d::pipelines::registration::RegistrationICP(*cloudSource,
//                                                                                 *cloudTarget,
//                                                                                 threshold,
//                                                                                 initTrans,
//                                                                                 open3d::pipelines::registration::TransformationEstimationPointToPoint(scaleIt = false),
//                                                                                 open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
//         return reg_result_ICP_PTP;
//     };


//     /// \summary ICP point-to-plane do need normals it's faster
//     /// \param initTrans The initial transformation to refine.
//     /// \param enable_robust_kernel The robust kernel can be enabled when the cloud is noisy
//     /// \param iterations Maximal number of iterations.
//     /// \param rel_fitness Boundary of fitness (the amount of corresponding points).
//     /// \param rel_RMSE Boundary of RMSE (the precision of the registration).
//     /// \param close_points Number of points to be estimate the normals(the more the better).
//     RS ICPPointToPlane(std::shared_ptr<PC> cloudSource,
//                     std::shared_ptr<PC> cloudTarget,
//                     Eigen::Matrix4d_u initTrans,
//                     float threshold,
//                     bool enable_robust_kernel = false,
//                     int iterations = 30,
//                     double rel_fitness = 1E-06,
//                     double rel_RMSE = 1E-06,
//                     int close_points = 100);

//     /// \summary ICP based on color, good for final refinement (e.g. to correct shifting)
//     /// \param initTrans The initial transformation to refine.
//     /// \param enable_robust_kernel The robust kernel can be enabled when the cloud is noisy
//     /// \param iterations Maximal number of iterations.
//     /// \param rel_fitness Boundary of fitness (the amount of corresponding points).
//     /// \param rel_RMSE Boundary of RMSE (the precision of the registration).
//     /// \param close_points Number of points to be estimate the normals(the more the better).
//     RS ICPColored(std::shared_ptr<PC> cloudSource,
//                 std::shared_ptr<PC> cloudTarget,
//                 Eigen::Matrix4d_u initTrans,
//                 float threshold,
//                 bool enable_robust_kernel = false,
//                 int iterations = 30,
//                 float if_kernel_lambda_geometric = 0.968f,
//                 double rel_fitness = 1E-06,
//                 double rel_RMSE = 1E-06,
//                 int close_points = 100)
//     {
//         // Estimate normals because ICP point-to-plane needs it
//         if (!(cloudSource->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(cloudSource, close_points, 0.1);
//         }
//         if (!(cloudTarget->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(cloudTarget, close_points, 0.1);
//         }

//         // To store results
//         RS reg_res;

//         // Call function with or without robust kernel
//         if (enable_robust_kernel)
//         {
//             std::shared_ptr<open3d::pipelines::registration::RobustKernel> loss_kernel = std::make_shared<open3d::pipelines::registration::L2Loss>();
//             loss_kernel->Weight(1.0);

//             // Call Open3D
//             reg_res = open3d::pipelines::registration::RegistrationICP(*cloudSource, *cloudTarget, threshold, initTrans,
//                                                                                     open3d::pipelines::registration::TransformationEstimationPointToPlane(loss_kernel),
//                                                                                     open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
//         }
//         else
//         {
//             // Call Open3D
//             reg_res = open3d::pipelines::registration::RegistrationICP(*cloudSource, *cloudTarget, threshold, initTrans,
//                                                                                     open3d::pipelines::registration::TransformationEstimationPointToPlane(),
//                                                                                     open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
//         }
//         return reg_res;
//     };

//     /// \summary RANSAC should be the first transformation, it can be refined later by a ICP.
//     /// \param voxel_size It needs a sub-sampling because it's very expensive.
//     /// \param with_ICP Include a ICP local refinement at the end.
//     /// \param similarity_threshold Loose (0) or strict (1).
//     /// \param checker_normal_degAngle Threshold of normal angle (the smallest, the more precise and quick).
//     /// \param ransac_n Number of points for RANSAC (min. 3)
//     /// \param mutual_filter True if the correspondence can be itself.
//     /// \param close_points Number of points to be estimate the normals(the more the better).
//     /// \param scaleIt Scale of not the source cloud to the target one.
//     /// \param convergence_max_iteration Maximal number of iterations.
//     /// \param convergence_confidence Boundary about the fitness of the registration (0.999 perfectly matched).
//     RS registrationRANSAC(std::shared_ptr<PC> cloudSource,
//                         std::shared_ptr<PC> cloudTarget,
//                         double voxel_size = 0.005,
//                         bool with_ICP = true,
//                         double similarity_threshold = 0.9,
//                         double checker_normal_degAngle = 30,
//                         int ransac_n = 3,
//                         bool mutual_filter = true,
//                         bool scaleIt = false,
//                         int convergence_max_iteration = 300000, // the number of iter
//                         float convergence_confidence = 0.999) // related to the fitness of the registration
//     {
//         // Store source and target cloudsd in array
//         PC_ptr low_res_clouds[2] = { cloudSource , cloudTarget };

//         // Voxel downsample PCs (5 mm is good -> 0.005 m)
//         low_res_clouds[0] = voxelDownSampling(low_res_clouds[0], voxel_size);
//         low_res_clouds[1] = voxelDownSampling(low_res_clouds[1], voxel_size);

//         // Compute threshold for evaluation
//         double distance_threshold = computeThreshold(low_res_clouds[0], low_res_clouds[1]);

//         // Estimate normals
//         double radius_normal = voxel_size * 2;
//         if (!(low_res_clouds[0]->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(low_res_clouds[0], 100, radius_normal, 30);
//         }
//         if (!(low_res_clouds[1]->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(low_res_clouds[1], 100, radius_normal, 30);
//         }

//         // Estimate FPFH features
//         std::vector<std::shared_ptr<open3d::pipelines::registration::Feature>> FPFH_features;
//         double radius_feature = voxel_size * 5;
//         for (PC_ptr cloud : low_res_clouds)
//         {
//             auto PC_FPFH = open3d::pipelines::registration::ComputeFPFHFeature(*cloud, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
//             FPFH_features.push_back(PC_FPFH);
//         }

//         // Pruning algorithms
//         auto check_dist = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
//         auto check_edge = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(similarity_threshold);
//         double pi = 3.14159265359;
//         double checker_normal_radAngle = (checker_normal_degAngle * (pi / 180)); // convert input angle in degrees to radians
//         auto check_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(checker_normal_radAngle); // circa 30 degrees of tolerance

//         // Execute registration with RANSAC
//         auto conv_crit = open3d::pipelines::registration::RANSACConvergenceCriteria(convergence_max_iteration, convergence_confidence);
//         RS reg_res_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(*low_res_clouds[0],
//                                                                                             *low_res_clouds[1],
//                                                                                             *FPFH_features[0],
//                                                                                             *FPFH_features[1],
//                                                                                             mutual_filter,
//                                                                                             distance_threshold,
//                                                                                             open3d::pipelines::registration::TransformationEstimationPointToPoint(scaleIt),
//                                                                                             ransac_n,
//                                                                                             { check_dist, check_edge, check_normal },
//                                                                                             conv_crit
//         );

//         // Ad ICP point-to-plane refinement following user input
//         if (with_ICP)
//         {
//             // ICP point-to-plane
//             RS reg_ICP = ICPPointToPlane(low_res_clouds[0],
//                                         low_res_clouds[1],
//                                         reg_res_RANSAC.transformation_,
//                                         voxel_size, // the tighest soft-coded value
//                                         false,
//                                         4000);
//             return reg_ICP;
//         }
//         else
//             return reg_res_RANSAC;
//     }

//     /// \summary Fast Global registration, with accurate parameters it can.
//     /// \param voxel_size size in meter for downsampling.
//     /// \param division_factor Division factor used for graduated non-convexity.
//     /// \param use_absolute_scale Measure distance in absolute scale (1) or in
//     /// scale relative to the diameter of the model (0).
//     /// \param decrease_mu Set
//     /// to `true` to decrease scale mu by division_factor for graduated
//     /// non-convexity.
//     /// \param maximum_correspondence_distance Maximum
//     /// correspondence distance (also see comment of USE_ABSOLUTE_SCALE).
//     /// \param iteration_number Maximum number of iterations.
//     /// \param tuple_scale Similarity measure used for tuples of feature points.
//     /// \param maximum_tuple_count Maximum numer of tuples.
//     /// \param checker_edge_length FPFH checker on maximum edge length
//     /// \param checker_normal_degAngle FPFH chcecker on normal angles deviation
//     RS registrationFast(std::shared_ptr<PC> cloudSource,
//                         std::shared_ptr<PC> cloudTarget,
//                         double voxel_size = 0.005,
//                         double division_factor = 1.4,
//                         bool use_absolute_scale = false,
//                         bool decrease_mu = true,
//                         float maximum_correspondence_distance = 0.025,
//                         int iteration_number = 64,
//                         double tuple_scale = 0.95,
//                         int maximum_tuple_count = 1000,
//                         double checker_edge_length = 0.9,
//                         double checker_normal_degAngle = 30) // in radians 0.5 ~ 30 deg
//     {
//         // Store source and target cloudsd in array
//         PC_ptr low_res_clouds[2] = { cloudSource , cloudTarget };

//         // Voxel downsample PCs (5 mm is good -> 0.005 m)
//         low_res_clouds[0] = voxelDownSampling(low_res_clouds[0], voxel_size);
//         low_res_clouds[1] = voxelDownSampling(low_res_clouds[1], voxel_size);

//         // Compute threshold for evaluation
//         double distance_threshold = computeThreshold(low_res_clouds[0], low_res_clouds[1]);

//         // Estimate normals
//         double radius_normal = voxel_size * 2;
//         if (!(low_res_clouds[0]->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(low_res_clouds[0], 100, radius_normal, 30);
//         }
//         if (!(low_res_clouds[1]->HasNormals()))
//         {
//             estimateUnstructuredPCDNormals(low_res_clouds[1], 100, radius_normal, 30);
//         }

//         // Estimate FPFH features
//         std::vector<std::shared_ptr<open3d::pipelines::registration::Feature>> FPFH_features;
//         double radius_feature = voxel_size * 5;
//         for (PC_ptr cloud : low_res_clouds)
//         {
//             auto PC_FPFH = open3d::pipelines::registration::ComputeFPFHFeature(*cloud, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
//             FPFH_features.push_back(PC_FPFH);
//         }

//         // Pruning algorithms
//         auto check_dist = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
//         auto check_edge = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(checker_edge_length);
//         double pi = 3.14159265359;
//         double checker_normal_radAngle = (checker_normal_degAngle * (pi / 180)); // convert input angle in degrees to radians
//         auto check_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(checker_normal_radAngle); // circa 30 degrees of tolerance

//         // Execute fast registration
//         auto fast_options = open3d::pipelines::registration::FastGlobalRegistrationOption(division_factor,
//                                                                                         use_absolute_scale,
//                                                                                         decrease_mu,
//                                                                                         maximum_correspondence_distance,
//                                                                                         iteration_number,
//                                                                                         tuple_scale,
//                                                                                         maximum_tuple_count);

//         RS reg_res_fast = open3d::pipelines::registration::FastGlobalRegistration(*low_res_clouds[0],
//                                                                                 *low_res_clouds[1],
//                                                                                 *FPFH_features[0],
//                                                                                 *FPFH_features[1],
//                                                                                 fast_options);

//         return reg_res_fast;
//     }
// }