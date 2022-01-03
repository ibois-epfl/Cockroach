#pragma once



#include <iostream>
#include <vector>

// // For Rhino.rhp
// #include "StdAfx.h"


// Open3D
#include <open3d/Open3D.h>
//
//// PCL
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

// Cilantro
#include <cilantro/utilities/point_cloud.hpp>


typedef open3d::geometry::PointCloud PC;
typedef open3d::pipelines::registration::RegistrationResult RS;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                          REGISTRATION                                                                                ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary Evaluate a possible first registration, it is mostly needed for a ICP.
/// \param threshold It's the maximal distance between two pair of points to be registered.
RS evaluateRegistration(std::shared_ptr<PC> cloudSource,
                        std::shared_ptr<PC> cloudTarget,
                        float threshold);

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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                       CLUSTER TECHNIQUES                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary HCS cluster: method based on normals and radius search.
/// \param voxel_size_search Size of voxel for point search, the smaller the more/smaller clusters it will produce.
/// \param normal_threshold_degree The threshold of tolerance for normal evaluation in degree angle.
/// \param min_cluster_size Minimum number of points in a cluster.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                                      float voxel_size_search = 0.1f,
                                                                                      double normal_threshold_degree = 2.0,
                                                                                      int min_cluster_size = 100,
                                                                                      bool color_point_cloud = true);

/// \summary HCS cluster : method based on normals and neighbour search.
/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
/// \param normal_threshold_degree The threshold of tolerance for normal evaluation in degree angle.
/// \param min_cluster_size Minimum number of points in a cluster.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentKSearch(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                                       int n_search = 30,
                                                                                       double normal_threshold_degree = 2,
                                                                                       int min_cluster_size = 100,
                                                                                       bool color_point_cloud = true);

/// \summary Spectral cluster : method based on Laplacian function [description to be compelted].
/// \param max_num_clusters Minimum number of clusters to be created.
/// \param k_search Number of neighbours to be searched.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_spectralCluster(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                             size_t max_num_clusters = 5,
                                                                             int k_search = 30,
                                                                             bool color_point_cloud = true);

/// \summary Euclidean cluster : method based on HCS cluster with distance function as main descriptor.
/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
/// \param dist_thresh The distance threshold of points to be cluster together.
/// \param min_cluster_size Minimum number of points in a cluster.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanKsearch(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                              int n_search = 30,
                                                                              float dist_thresh = 0.02,
                                                                              int min_cluster_size = 30,
                                                                              bool color_point_cloud = true);

/// \summary Euclidean cluster : method based on HCS cluster with distance function as main descriptor.
/// \param voxel_size_search Size of voxel for point search, the smaller the more/smaller clusters it will produce.
/// \param dist_thresh The distance threshold of points to be cluster together.
/// \param min_cluster_size Minimum number of points in a cluster.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                             float voxel_size_search,
                                                                             float dist_thresh,
                                                                             int min_cluster_size,
                                                                             bool color_point_cloud = true);

/// \summary K-means cluster : iteratively finds spherical clusters of points (usually high-dimensional), where cluster affinity is based on a distance to the cluster center.
/// \param n_cluster Number of cluster to cast.
/// \param max_iter Maximal iterations.
/// \param use_k_tree Use or not the k-tree search
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_kMeans(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                    size_t n_cluster = 250,
                                                                    size_t max_iter = 100,
                                                                    bool use_k_tree = true,
                                                                    bool color_point_cloud = true);

/// \summary Mean shift clustering aims to discover �blobs� in a smooth density of samples.
/// \param kernel_radius Radius of the kernel search.
/// \param max_iter Max number of iterations.
/// \param cluster_tol Tolerance of the clustering.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_meanShift(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                       float kernel_radius = 2.0f,
                                                                       int max_iter = 5000,
                                                                       float cluster_tol = 0.2f,
                                                                       bool color_point_cloud = true);

/// \summary Cluster points with the same color.
/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
/// \param dist_thresh The color distance threshold of points to be cluster together.
/// \param min_cluster_size Minimum number of points in a cluster.
/// \param color_point_cloud Apply random color to new cluster or leave original colors.
std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_color(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                   int n_search = 30,
                                                                   float dist_tresh = 0.01f,
                                                                   int min_cluster_size = 30,
                                                                   bool color_point_cloud = true);

/// \summary Segment the biggest plane in the scene and retrieve the outlier points.
/// \param distanceThreshold The istance threshold from the plane to include close points.
/// \param numIterations Max number of iterations.
std::shared_ptr<open3d::geometry::PointCloud> planeSegmentation(std::shared_ptr <open3d::geometry::PointCloud> cloud,
                                                                float distanceThreshold = 0.01f,
                                                                int ransacN = 3,
                                                                int numIterations = 1000);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                        VISUALIZATION                                                                                 ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary Standard visualization.
void visualize_standard(std::shared_ptr<open3d::geometry::PointCloud> cloud);

/// \summary Visualize two clouds for registration.
void visualize_bicolor(std::shared_ptr<open3d::geometry::PointCloud> cloudSource, std::shared_ptr<open3d::geometry::PointCloud> cloudTarget, Eigen::Matrix4d transformation);
void visualize_bicolor(std::shared_ptr<open3d::geometry::PointCloud> cloudSource, std::shared_ptr<open3d::geometry::PointCloud> cloudTarget);

/// \summary Visualize mesh with normals.
void MeshNormalVisualize(std::shared_ptr<open3d::geometry::TriangleMesh> mesh);

/// \summary Visualize cloud with normals.
void visualize_cloudNormals(std::shared_ptr<open3d::geometry::PointCloud> cloud);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                          UTILITIES                                                                                   ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary To compute the resolution of the point cloud: in other words, the average distance between pointss of the same cloud.
float computePCResolution(std::shared_ptr <open3d::geometry::PointCloud> cloud);

/// \summary Downsample the cloud by grid size.
std::shared_ptr<open3d::geometry::PointCloud> voxelDownSampling(std::shared_ptr<open3d::geometry::PointCloud> cloud, float VoxelSize);

/// \summary Downsample the cloud by random sub-sampling given a target number of points.
std::shared_ptr<open3d::geometry::PointCloud> uniformDownsample(std::shared_ptr<open3d::geometry::PointCloud> cloud, int targetOfPoints);

/// \summary Outlier removal by statistical analysis of points.
std::shared_ptr<open3d::geometry::PointCloud> statisticalOutlierRemoval(std::shared_ptr<open3d::geometry::PointCloud> cloud, int nbNeighbors = 20, double stdRatio = 2.0);

/// \summary Retrieve current directory of exe file.
std::string getCurrentDirectory();

/// \summary Import Open3d point cloud.
std::shared_ptr<open3d::geometry::PointCloud> importCloud(std::string FILE_NAME_);

/// \summary Oriented bounding box following the object axes.
std::shared_ptr<open3d::geometry::OrientedBoundingBox> computeOrientedBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> cloud);

/// \summary Compute the maximal distance between to points of two clouds to be registered.
double computeThreshold(std::shared_ptr<PC> cloudSource, std::shared_ptr<PC> cloudTarget);

/// \summary Estimate normals and orient them on the same direction.
void estimateUnstructuredPCDNormals(std::shared_ptr<open3d::geometry::PointCloud> cloud,
                                    int numberOfCPUser,
                                    double radius_search = 0.1,
                                    int iter = 30);
/// <summary>
/// Substract two point clouds
/// </summary>
/// <param name="cloud_one"> feed the smallest cloud (cutter) </param>
/// <param name="cloud_two"> feed the largest cloud (to be cut) </param>
/// <param name="Ksearch_rad"> radius of tolerance for substracting </param>
/// <returns></returns>
std::shared_ptr<open3d::geometry::PointCloud> substractCloud(std::shared_ptr<open3d::geometry::PointCloud> cloud_one,
                                                             std::shared_ptr<open3d::geometry::PointCloud> cloud_two,
                                                             double Ksearch_rad);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                   CONVERSION FROM LIB TO LIB                                                                         ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \summary Cilantro > OP3.
void convert_CilantroToOpen3DCloud(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f, std::shared_ptr<PC> open3d_cloud_d);

/// \summary OP3 > Cilantro.
void convert_Open3DToCilantroCloud(std::shared_ptr<PC> open3d_cloud_d, std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f);

///// \summary PCLXYZ > OP3.
//void convert_PCLXYZToOpen3DCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud);
//
///// \summary PCLNormal > OP3.
//void convert_PCLToOpen3DNormal(pcl::PointCloud<pcl::Normal>::Ptr pcl_normal, std::vector<Eigen::Vector3d> open3d_normal);
//
///// \summary PCLXYZRGB > OP3.
//void convert_PCLXYZRGBToOpen3DCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud);
//
///// \summary PCLXYZRGBA > OP3.
//void convert_PCLXYZRGBAToOpen3Cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud);
//
///// \summary PCLXYZRGBNormal > OP3.
//void convert_PCLXYZRGBNormalToOpen3DCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud);
//
///// \summary OP3 > PCLXYZ.
//void convert_Open3DToPCLXYZCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
//
///// \summary OP3 > PCLNormal.
//void convert_Open3DToPCLNormal(std::vector<Eigen::Vector3d> open3d_normal, pcl::PointCloud<pcl::Normal>::Ptr pcl_normal);
//
///// \summary OP3 > PCLXYZRGB.
//void convert_Open3DToPCLXYZRGBCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud);
//
///// \summary OP3 > PCLXYZRGBA.
//void convert_Open3DToPCLXYZRGBACloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud);
//
///// \summary OP3 > PCLXYZRGBNormal.
//void convert_Open3DToPCLXYZRGBNormalCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                           MESHING                                                                                    ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
