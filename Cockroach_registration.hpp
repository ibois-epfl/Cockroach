#pragma once

#include <iostream>
#include <vector>

#include <open3d/Open3D.h> // Open3D
#include <cilantro/utilities/point_cloud.hpp> // Cilantro

typedef open3d::geometry::PointCloud PC;
typedef open3d::pipelines::registration::RegistrationResult RS;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                          REGISTRATION                                                                                ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary Evaluate a possible first registration, it is mostly needed for a ICP.
/// \param threshold It's the maximal distance between two pair of points to be registered.
RS evaluateRegistration(std::shared_ptr<PC> cloudSource,
                        std::shared_ptr<PC> cloudTarget,
                        float threshold){
    // Call the open3d method
    auto registration_result = open3d::pipelines::registration::EvaluateRegistration(*cloudSource, *cloudTarget, threshold);
    return registration_result;
};

/// \summary ICP point-to-point do not need normals but it's more expensive and prone to shifting on the same plane
/// \param initTrans The initial transformation to refine.
/// \param scaleIt Scale of not the source cloud to the target one.
/// \param iterations Maximal number of iterations.
/// \param rel_fitness Boundary of fitness (the amount of corresponding points).
/// \param rel_RMSE Boundary of RMSE (the precision of the registration).
RS ICPPointToPoint(std::shared_ptr<PC> cloudSource,
                   std::shared_ptr<PC> cloudTarget,
                   Eigen::Matrix4d_u initTrans,
                   float threshold,
                   bool scaleIt = false,
                   int iterations = 30,
                   double rel_fitness = 1E-06,
                   double rel_RMSE = 1E-06);

/// \summary ICP point-to-plane do need normals it's faster
/// \param initTrans The initial transformation to refine.
/// \param enable_robust_kernel The robust kernel can be enabled when the cloud is noisy
/// \param iterations Maximal number of iterations.
/// \param rel_fitness Boundary of fitness (the amount of corresponding points).
/// \param rel_RMSE Boundary of RMSE (the precision of the registration).
/// \param close_points Number of points to be estimate the normals(the more the better).
RS ICPPointToPlane(std::shared_ptr<PC> cloudSource,
                   std::shared_ptr<PC> cloudTarget,
                   Eigen::Matrix4d_u initTrans,
                   float threshold,
                   bool enable_robust_kernel = false,
                   int iterations = 30,
                   double rel_fitness = 1E-06,
                   double rel_RMSE = 1E-06,
                   int close_points = 100);

/// \summary ICP based on color, good for final refinement (e.g. to correct shifting)
/// \param initTrans The initial transformation to refine.
/// \param enable_robust_kernel The robust kernel can be enabled when the cloud is noisy
/// \param iterations Maximal number of iterations.
/// \param rel_fitness Boundary of fitness (the amount of corresponding points).
/// \param rel_RMSE Boundary of RMSE (the precision of the registration).
/// \param close_points Number of points to be estimate the normals(the more the better).
RS ICPColored(std::shared_ptr<PC> cloudSource,
              std::shared_ptr<PC> cloudTarget,
              Eigen::Matrix4d_u initTrans,
              float threshold,
              bool enable_robust_kernel = false,
              int iterations = 30,
              float if_kernel_lambda_geometric = 0.968f,
              double rel_fitness = 1E-06,
              double rel_RMSE = 1E-06,
              int close_points = 100);

/// \summary RANSAC should be the first transformation, it can be refined later by a ICP.
/// \param voxel_size It needs a sub-sampling because it's very expensive.
/// \param with_ICP Include a ICP local refinement at the end.
/// \param similarity_threshold Loose (0) or strict (1).
/// \param checker_normal_degAngle Threshold of normal angle (the smallest, the more precise and quick).
/// \param ransac_n Number of points for RANSAC (min. 3)
/// \param mutual_filter True if the correspondence can be itself.
/// \param close_points Number of points to be estimate the normals(the more the better).
/// \param scaleIt Scale of not the source cloud to the target one.
/// \param convergence_max_iteration Maximal number of iterations.
/// \param convergence_confidence Boundary about the fitness of the registration (0.999 perfectly matched).
RS registrationRANSAC(std::shared_ptr<PC> cloudSource,
                      std::shared_ptr<PC> cloudTarget,
                      double voxel_size = 0.005,
                      bool with_ICP = true,
                      double similarity_threshold = 0.9,
                      double checker_normal_degAngle = 30,
                      int ransac_n = 3,
                      bool mutual_filter = true,
                      bool scaleIt = false,
                      int convergence_max_iteration = 300000, // the number of iter
                      float convergence_confidence = 0.999); // related to the fitness of the registration

/// \summary Fast Global registration, with accurate parameters it can.
/// \param voxel_size size in meter for downsampling.
/// \param division_factor Division factor used for graduated non-convexity.
/// \param use_absolute_scale Measure distance in absolute scale (1) or in
/// scale relative to the diameter of the model (0).
/// \param decrease_mu Set
/// to `true` to decrease scale mu by division_factor for graduated
/// non-convexity.
/// \param maximum_correspondence_distance Maximum
/// correspondence distance (also see comment of USE_ABSOLUTE_SCALE).
/// \param iteration_number Maximum number of iterations.
/// \param tuple_scale Similarity measure used for tuples of feature points.
/// \param maximum_tuple_count Maximum numer of tuples.
/// \param checker_edge_length FPFH checker on maximum edge length
/// \param checker_normal_degAngle FPFH chcecker on normal angles deviation
RS registrationFast(std::shared_ptr<PC> cloudSource,
                      std::shared_ptr<PC> cloudTarget,
                      double voxel_size = 0.005,
                      double division_factor = 1.4,
                      bool use_absolute_scale = false,
                      bool decrease_mu = true,
                      float maximum_correspondence_distance = 0.025,
                      int iteration_number = 64,
                      double tuple_scale = 0.95,
                      int maximum_tuple_count = 1000,
                      double checker_edge_length = 0.9,
                      double checker_normal_degAngle = 30); // in radians 0.5 ~ 30 deg
