#include <iostream>

// // For Rhino.rhp
// #include "StdAfx.h"

// Open3D
#include "open3d/Open3D.h"
#include <open3d/pipelines/registration/RobustKernel.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>

// Cilantro
#include <cilantro/utilities/point_cloud.hpp>

// Personal
#include "Cockroach.h"


typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
typedef std::shared_ptr<PC> PC_ptr;
typedef open3d::pipelines::registration::RegistrationResult RS;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                          REGISTRATION                                                                                ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                       LOCAL REGISTRATION                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////
///        EVALUATE REGISTRATION        ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html */

RS evaluateRegistration(std::shared_ptr<PC> cloudSource,
                        std::shared_ptr<PC> cloudTarget,
                        float threshold)
{
    // Call the open3d method
    auto registration_result = open3d::pipelines::registration::EvaluateRegistration(*cloudSource, *cloudTarget, threshold);
    return registration_result;
};


///////////////////////////////////////////
///        ICP : POINT-TO-POINT         ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/robust_kernels.html */

/* NO NORMALS: The ICP point-to-point do not need normals*/

/* It can still mismatch geometries on the same plane (shifting is possible)*/

RS ICPPointToPoint(std::shared_ptr<PC> cloudSource,
                   std::shared_ptr<PC> cloudTarget,
                   Eigen::Matrix4d_u initTrans,
                   float threshold,
                   bool scaleIt,
                   int iterations,
                   double rel_fitness,
                   double rel_RMSE)
{
    // Call Open3D method
    auto reg_result_ICP_PTP = open3d::pipelines::registration::RegistrationICP(*cloudSource,
                                                                               *cloudTarget,
                                                                               threshold,
                                                                               initTrans,
                                                                               open3d::pipelines::registration::TransformationEstimationPointToPoint(scaleIt = false),
                                                                               open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
    return reg_result_ICP_PTP;
};


///////////////////////////////////////////
///     ICP : POINT-TO-PLANE + LOSS     ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/robust_kernels.html */

/* NORMALS: The ICP point-to-point do need normals*/

/* LOSS FUNCTIONS: It can be applied to any particular registration/or not target. e.g. It deals by adding tolerances to the presence of outlier in the cloud.
 * To call it in the ICP registration we need to  add the TransformationEEstimationPointToPlane(loss), where "loss" is a given loss function (also called robust kernel),
 * there are much more, you can find the list in the method block: */

RS ICPPointToPlane(std::shared_ptr<PC> cloudSource,
                   std::shared_ptr<PC> cloudTarget,
                   Eigen::Matrix4d_u initTrans,
                   float threshold, // maximum correspondence points - pair distance
                   bool enable_robust_kernel,
                   int iterations,
                   double rel_fitness,
                   double rel_RMSE,
                   int close_points)
{
    // Estimate normals because ICP point-to-plane needs it
    if (!(cloudSource->HasNormals()))
    {
        estimateUnstructuredPCDNormals(cloudSource, close_points, 0.1);
    }
    if (!(cloudTarget->HasNormals()))
    {
        estimateUnstructuredPCDNormals(cloudTarget, close_points, 0.1);
    }

    // To store results
    RS reg_res;

    // Call function with or without robust kernel
    if (enable_robust_kernel)
    {
        std::shared_ptr<open3d::pipelines::registration::RobustKernel> loss_kernel = std::make_shared<open3d::pipelines::registration::L2Loss>();
        loss_kernel->Weight(1.0);

        // Call Open3D
        reg_res = open3d::pipelines::registration::RegistrationICP(*cloudSource, *cloudTarget, threshold, initTrans,
                                                                                   open3d::pipelines::registration::TransformationEstimationPointToPlane(loss_kernel),
                                                                                   open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
    }
    else
    {
        // Call Open3D
        reg_res = open3d::pipelines::registration::RegistrationICP(*cloudSource, *cloudTarget, threshold, initTrans,
                                                                                   open3d::pipelines::registration::TransformationEstimationPointToPlane(),
                                                                                   open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
    }
    return reg_res;
};


///////////////////////////////////////////
///            COLORED ICP              ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/colored_pointcloud_registration.html */

/* NORMALS: The ICP point-to-point do need normals*/

/* This algorithm is more accurate and more robust thanother PC registration alorithmss, while the running speed is comparable to that of ICP registration types*/


RS ICPColored(std::shared_ptr<PC> cloudSource,
              std::shared_ptr<PC> cloudTarget,
              Eigen::Matrix4d_u initTrans,
              float threshold,
              bool enable_robust_kernel,
              int iterations,
              float if_kernel_lambda_geometric,
              double rel_fitness,
              double rel_RMSE,
              int close_points)
{
    // Estimate normals because ICP point-to-plane needs it
    if (!(cloudSource->HasNormals()))
    {
        estimateUnstructuredPCDNormals(cloudSource, close_points, 0.1);
    }
    if (!(cloudTarget->HasNormals()))
    {
        estimateUnstructuredPCDNormals(cloudTarget, close_points, 0.1);
    }

    // To store results
    RS reg_res;

    // Call function with or without robust kernel
    if (enable_robust_kernel)
    {
        std::shared_ptr<open3d::pipelines::registration::RobustKernel> loss_kernel = std::make_shared<open3d::pipelines::registration::L2Loss>();
        loss_kernel->Weight(1.0);

        // Call Open3D
        reg_res = open3d::pipelines::registration::RegistrationColoredICP(*cloudSource, *cloudTarget, threshold, initTrans,
                                                                                   open3d::pipelines::registration::TransformationEstimationForColoredICP(if_kernel_lambda_geometric, loss_kernel),
                                                                                   open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
    }
    else
    {
        // Call Open3D
        reg_res = open3d::pipelines::registration::RegistrationColoredICP(*cloudSource, *cloudTarget, threshold, initTrans,
                                                                                   open3d::pipelines::registration::TransformationEstimationForColoredICP(),
                                                                                   open3d::pipelines::registration::ICPConvergenceCriteria(rel_fitness, rel_RMSE, iterations));
    }
    return reg_res;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                     GLOBAL  REGISTRATION                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////
///    RANSAC FEATURE GL REGISTRATION   ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/global_registration.html */

/* This is a global registration in five steps:
 * 1) downsize the pointcloud for less computational time;
 * 2) evaluate normals and FPFH (33-ddimensional vector) which characterize the geometric features of each and single point;
 * 3) pruning algorithms: based on FPFH correspondences to get rid of possible alternatives based on distance threshold, edge length, normal angles;
 * 4) run the RANSAC-based registration;
 * 5) refine locally by ICP point-to-plane;
 */

RS registrationRANSAC(std::shared_ptr<PC> cloudSource,
                      std::shared_ptr<PC> cloudTarget,
                      double voxel_size,
                      bool with_ICP,
                      double similarity_threshold,
                      double checker_normal_degAngle, 
                      int ransac_n, 
                      bool mutual_filter, 
                      bool scaleIt,
                      int convergence_max_iteration, 
                      float convergence_confidence) 
{
    // Store source and target cloudsd in array
    PC_ptr low_res_clouds[2] = { cloudSource , cloudTarget };

    // Voxel downsample PCs (5 mm is good -> 0.005 m)
    low_res_clouds[0] = voxelDownSampling(low_res_clouds[0], voxel_size);
    low_res_clouds[1] = voxelDownSampling(low_res_clouds[1], voxel_size);

    // Compute threshold for evaluation
    double distance_threshold = computeThreshold(low_res_clouds[0], low_res_clouds[1]);

    // Estimate normals
    double radius_normal = voxel_size * 2;
    if (!(low_res_clouds[0]->HasNormals()))
    {
        estimateUnstructuredPCDNormals(low_res_clouds[0], 100, radius_normal, 30);
    }
    if (!(low_res_clouds[1]->HasNormals()))
    {
        estimateUnstructuredPCDNormals(low_res_clouds[1], 100, radius_normal, 30);
    }

    // Estimate FPFH features
    std::vector<std::shared_ptr<open3d::pipelines::registration::Feature>> FPFH_features;
    double radius_feature = voxel_size * 5;
    for (PC_ptr cloud : low_res_clouds)
    {
        auto PC_FPFH = open3d::pipelines::registration::ComputeFPFHFeature(*cloud, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
        FPFH_features.push_back(PC_FPFH);
    }

    // Pruning algorithms
    auto check_dist = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto check_edge = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(similarity_threshold);
    double pi = 3.14159265359;
    double checker_normal_radAngle = (checker_normal_degAngle * (pi / 180)); // convert input angle in degrees to radians
    auto check_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(checker_normal_radAngle); // circa 30 degrees of tolerance

    // Execute registration with RANSAC
    auto conv_crit = open3d::pipelines::registration::RANSACConvergenceCriteria(convergence_max_iteration, convergence_confidence);
    RS reg_res_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(*low_res_clouds[0],
                                                                                           *low_res_clouds[1],
                                                                                           *FPFH_features[0],
                                                                                           *FPFH_features[1],
                                                                                           mutual_filter,
                                                                                           distance_threshold,
                                                                                           open3d::pipelines::registration::TransformationEstimationPointToPoint(scaleIt),
                                                                                           ransac_n,
                                                                                           { check_dist, check_edge, check_normal },
                                                                                           conv_crit
    );

    // Ad ICP point-to-plane refinement following user input
    if (with_ICP)
    {
        // ICP point-to-plane
        RS reg_ICP = ICPPointToPlane(low_res_clouds[0],
                                     low_res_clouds[1],
                                     reg_res_RANSAC.transformation_,
                                     voxel_size, // the tighest soft-coded value
                                     false,
                                     4000);
        return reg_ICP;
    }
    else
        return reg_res_RANSAC;
}


///////////////////////////////////////////
///     FAST FEATURE GL REGISTRATION    ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/pipelines/global_registration.html */

/* The RANSAC based global registration solution may take a long time due to countless model proposals and evaluations. [Zhou2016] introduced a faster approach that
 * quickly optimizes line process weights of few correspondences. As there is no model proposal and evaluation invlolved for each iteration, this approach can save a lot
 * of computational time. With proper configuration, the accuracy of fast global registration is even comparable to ICP. */

RS registrationFast(std::shared_ptr<PC> cloudSource,
                      std::shared_ptr<PC> cloudTarget,
                      double voxel_size,
                      double division_factor,
                      bool use_absolute_scale,
                      bool decrease_mu,
                      float maximum_correspondence_distance,
                      int iteration_number,
                      double tuple_scale,
                      int maximum_tuple_count,
                      double checker_edge_length,
                      double checker_normal_degAngle)
{
    // Store source and target cloudsd in array
    PC_ptr low_res_clouds[2] = { cloudSource , cloudTarget };

    // Voxel downsample PCs (5 mm is good -> 0.005 m)
    low_res_clouds[0] = voxelDownSampling(low_res_clouds[0], voxel_size);
    low_res_clouds[1] = voxelDownSampling(low_res_clouds[1], voxel_size);

    // Compute threshold for evaluation
    double distance_threshold = computeThreshold(low_res_clouds[0], low_res_clouds[1]);

    // Estimate normals
    double radius_normal = voxel_size * 2;
    if (!(low_res_clouds[0]->HasNormals()))
    {
        estimateUnstructuredPCDNormals(low_res_clouds[0], 100, radius_normal, 30);
    }
    if (!(low_res_clouds[1]->HasNormals()))
    {
        estimateUnstructuredPCDNormals(low_res_clouds[1], 100, radius_normal, 30);
    }

    // Estimate FPFH features
    std::vector<std::shared_ptr<open3d::pipelines::registration::Feature>> FPFH_features;
    double radius_feature = voxel_size * 5;
    for (PC_ptr cloud : low_res_clouds)
    {
        auto PC_FPFH = open3d::pipelines::registration::ComputeFPFHFeature(*cloud, open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
        FPFH_features.push_back(PC_FPFH);
    }

    // Pruning algorithms
    auto check_dist = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto check_edge = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(checker_edge_length);
    double pi = 3.14159265359;
    double checker_normal_radAngle = (checker_normal_degAngle * (pi / 180)); // convert input angle in degrees to radians
    auto check_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(checker_normal_radAngle); // circa 30 degrees of tolerance

    // Execute fast registration
    auto fast_options = open3d::pipelines::registration::FastGlobalRegistrationOption(division_factor,
                                                                                      use_absolute_scale,
                                                                                      decrease_mu,
                                                                                      maximum_correspondence_distance,
                                                                                      iteration_number,
                                                                                      tuple_scale,
                                                                                      maximum_tuple_count);

    RS reg_res_fast = open3d::pipelines::registration::FastGlobalRegistration(*low_res_clouds[0],
                                                                              *low_res_clouds[1],
                                                                              *FPFH_features[0],
                                                                              *FPFH_features[1],
                                                                              fast_options);

    return reg_res_fast;
}
