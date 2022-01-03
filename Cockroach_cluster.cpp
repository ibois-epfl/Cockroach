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
#include <cilantro/utilities/io_utilities.hpp>
#include <cilantro/utilities/ply_io.hpp>
#include <cilantro/utilities/timer.hpp>
#include <cilantro/clustering/connected_component_extraction.hpp>
#include <cilantro/utilities/nearest_neighbor_graph_utilities.hpp>
#include <cilantro/clustering/spectral_clustering.hpp>
#include <cilantro/clustering/mean_shift.hpp>

// Personal
#include "Cockroach.h"


// Shorthands
typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
typedef std::shared_ptr<PC> PC_ptr;
typedef open3d::pipelines::registration::RegistrationResult RS;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                       CLUSTER TECHNIQUES                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* The goal of point cloud segmentation is to identify points in a cloud with similar features for clustering into their respective regions */

///////////////////////////////////////////
///             HCS CLUSTER             ///
///////////////////////////////////////////

// Source:
/* https://github.com/kzampog/cilantro/blob/master/examples/connected_component_extraction.cpp */

/* "Highly Connected Component Cluster"(?) or "connected-component-extractio-cluster" divides the pointcloud in regions separeted by one particular geometric descriptor.
 * To our best knowledge this is the fastest and unique function in all PC processing libraries.
 * It supports arbitrary point-wise similarity functions and it allows connected components to be found within organized point cloud data, given a comparison function:
 * in this case is based on normal evaluation. This register different cluster regions with abrupt change in normal orientation or similar orientation.
 */

/* NORMALS are needed for this particular cluster*/

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                        float voxel_size_search,
                                                                        double normal_threshold_degree,
                                                                        int min_cluster_size,
                                                                        bool color_point_cloud)
{
    // Point search by radius
    cilantro::RadiusNeighborhoodSpecification<float> nh(voxel_size_search * voxel_size_search);

    // Normal evaluate
    cilantro::NormalsProximityEvaluator<float, 3> ev(cilantro_cloud_f->normals, (float)(normal_threshold_degree * (M_PI / 180)));
    // Perform clustering/segmentation
    cilantro::ConnectedComponentExtraction3f<> cce(cilantro_cloud_f->points);
    cce.segment(nh, ev, min_cluster_size, cilantro_cloud_f->size());

    // Build a color map
    size_t num_labels = cce.getNumberOfClusters();
    const auto& labels = cce.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = cce.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = cce.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for ( int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentKSearch(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                          int n_search,
                                                                          double normal_threshold_degree,
                                                                          int min_cluster_size,
                                                                          bool color_point_cloud)
{
    // Point search by k neighbours
    cilantro::KNNNeighborhoodSpecification<int> nk_search(n_search);

    // Normal evaluate
    cilantro::NormalsProximityEvaluator<float, 3> ev(cilantro_cloud_f->normals, (float)(normal_threshold_degree * (M_PI / 180)));

    // Perform clustering/segmentation
    cilantro::ConnectedComponentExtraction3f<> cce(cilantro_cloud_f->points);
    cce.segment(nk_search, ev, min_cluster_size, cilantro_cloud_f->size());

    // Build a color map
    size_t num_labels = cce.getNumberOfClusters();
    const auto& labels = cce.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = cce.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = cce.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};
///////////////////////////////////////////
///         EUCLIDEAN CLUSTER           ///
///////////////////////////////////////////

// Source:
/* https://github.com/kzampog/cilantro/blob/master/examples/connected_component_extraction.cpp */

/* From [cilantro: A Lean, Versatile, and Efficient Library for Point Cloud Data Processing, 2018]:
 * We note that common segmentation tasks such as extracting Euclidean (see PCL�s EuclideanClusterExtraction) or smooth (see PCL�s
RegionGrowing) segments can be straightforwardly cast as connected component segmentation under different similarity metrics.

is a greedy region growing algorithm based on nearest neighbors. Cluster affinity is based on a distance to any point of a cluster (cluster tolerance parameter).*/

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanKsearch(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                              int n_search,
                                                                              float dist_thresh,
                                                                              int min_cluster_size,
                                                                              bool color_point_cloud)
{
    // Point search by k neighbours
    cilantro::KNNNeighborhoodSpecification<int> nk_search(n_search);

    // Distance evaluator
    cilantro::PointsProximityEvaluator<float> pe(dist_thresh);

    // Perform clustering/segmentation
    cilantro::ConnectedComponentExtraction3f<> cce(cilantro_cloud_f->points);
    cce.segment(nk_search, pe, min_cluster_size, cilantro_cloud_f->size());

    // Build a color map
    size_t num_labels = cce.getNumberOfClusters();
    const auto& labels = cce.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = cce.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = cce.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                             float voxel_size_search,
                                                                             float dist_thresh,
                                                                             int min_cluster_size,
                                                                             bool color_point_cloud)
{
    // Point search by radius
    cilantro::RadiusNeighborhoodSpecification<float> nh(voxel_size_search * voxel_size_search);

    // Distance evaluator
    cilantro::PointsProximityEvaluator<float> pe(dist_thresh);

    // Perform clustering/segmentation
    cilantro::ConnectedComponentExtraction3f<> cce(cilantro_cloud_f->points);
    cce.segment(nh, pe, min_cluster_size, cilantro_cloud_f->size());

    // Build a color map
    size_t num_labels = cce.getNumberOfClusters();
    const auto& labels = cce.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = cce.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = cce.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};

///////////////////////////////////////////
///          SPECTRAL CLUSTER           ///
///////////////////////////////////////////

// Source:
/* https://github.com/kzampog/cilantro/blob/master/examples/spectral_clustering.cpp */

/* It is based on Laplacian functions*/

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_spectralCluster(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                             size_t max_num_clusters,
                                                                             int k_search,
                                                                             bool color_point_cloud )
{
    // Variables
    cilantro::VectorSet3f points = cilantro_cloud_f->points;
    Eigen::SparseMatrix<float> affinities;

    // Build neighbourhood graph
    auto nn = cilantro::KDTree3f<>(points).search(points, cilantro::KNNNeighborhoodSpecification<>(k_search));
    affinities = cilantro::getNNGraphFunctionValueSparseMatrix(nn, cilantro::RBFKernelWeightEvaluator<float>(), true);

    // Execute spectral cluster
    cilantro::SpectralClustering<float> sc(affinities, max_num_clusters, true, cilantro::GraphLaplacianType::NORMALIZED_RANDOM_WALK);

    // Build a color map
    size_t num_labels = sc.getNumberOfClusters();
    const auto& labels = sc.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = sc.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = sc.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
}

///////////////////////////////////////////
///           KMEANS CLUSTER            ///
///////////////////////////////////////////

// Source:
/* https://github.com/kzampog/cilantro/blob/master/examples/kmeans.cpp */

/* iteratively finds spherical clusters of points (usually high-dimensional), where cluster affinity is based on a distance to the cluster center.
 * From a math point of view: it chooses centroids that will minimize the squared distances of cluster points to the centroid -
 * and as mentioned, each points belongs to the cluster with the nearest centroid. */

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_kMeans(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                       size_t n_cluster,
                                                       size_t max_iter,
                                                       bool use_k_tree,
                                                       bool color_point_cloud)
{
    // Variables
    float tol = std::numeric_limits<float>::epsilon();

    // k-means on point coordinates
    cilantro::KMeans3f<> kmc(cilantro_cloud_f->points);

    // Call k-means cluster
    kmc.cluster(n_cluster, max_iter, tol, use_k_tree);

    // Build a color map
    size_t num_labels = kmc.getNumberOfClusters();
    const auto& labels = kmc.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = kmc.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = kmc.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
}

///////////////////////////////////////////
///         MEAN-SHIFT CLUSTER          ///
///////////////////////////////////////////

// Source:
/* https://github.com/kzampog/cilantro/blob/master/examples/mean_shift.cpp */

/* From: https://scikit-learn.org/stable/modules/generated/sklearn.cluster.MeanShift.html#sklearn.cluster.MeanShift
 * Mean shift clustering aims to discover �blobs� in a smooth density of samples. It is a centroid-based algorithm, which works by updating candidates for centroids
 * to be the mean of the points within a given region. These candidates are then filtered in a post-processing stage to eliminate near-duplicates to form the final set of centroids.
 */

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_meanShift(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                       float kernel_radius,
                                                                       int max_iter,
                                                                       float cluster_tol,
                                                                       bool color_point_cloud)
{
    // Execute k-mean shift
    cilantro::MeanShift3f<> ms(cilantro_cloud_f->points);
    ms.cluster(kernel_radius,
               max_iter,
               cluster_tol,
               1E-07,
               cilantro::UnityWeightEvaluator<float>());

    // Build a color map
    size_t num_labels = ms.getNumberOfClusters();
    const auto& labels = ms.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = ms.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of clister per point
    auto pt_index_cluster = ms.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};

///////////////////////////////////////////
///            COLOR CLUSTER            ///
///////////////////////////////////////////

std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_color(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
                                                                   int n_search,
                                                                   float dist_tresh,
                                                                   int min_cluster_size,
                                                                   bool color_point_cloud)
{
    // PC colors
    cilantro::ConstDataMatrixMap<float, 3> pc_col(cilantro_cloud_f->colors);

    // Point search by k neighbours
    cilantro::KNNNeighborhoodSpecification<int> nk_search(n_search);

    // colors proximity evaluate
    cilantro::ColorsProximityEvaluator<float> ce(pc_col, dist_tresh);

    // Perform clustering/segmentation
    cilantro::ConnectedComponentExtraction3f<> cce(cilantro_cloud_f->points);
    cce.segment(nk_search, ce, min_cluster_size, cilantro_cloud_f->size());

    // Build a color map
    size_t num_labels = cce.getNumberOfClusters();
    const auto& labels = cce.getPointToClusterIndexMap();
    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }

    // No label
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    // Create a new unifed colored cloud
    std::shared_ptr<cilantro::PointCloud3f> cloud_seg(new cilantro::PointCloud3f(cilantro_cloud_f->points, cilantro_cloud_f->normals, colors));

    ///////////////////////

    // Create different cloud per cluster
    num_labels = cce.getNumberOfClusters();
    std::cout << "the number of clusters " + std::to_string(num_labels) << std::endl;

    // Get index of cluster per point
    auto pt_index_cluster = cce.getPointToClusterIndexMap();

    // Vector to host individual PCs
    std::vector<std::shared_ptr<cilantro::PointCloud3f>> ptr_clouds_list;

    // Iterate through cloud and create seperate cluster
    auto ptt = cilantro_cloud_f->points;
    int n_pt = (int)ptt.cols();
    auto col = cilantro_cloud_f->colors;
    auto col_seg = cloud_seg->colors;
    auto nor = cilantro_cloud_f->normals;

    for (int i = 0; i < num_labels; i++)
    {
        std::shared_ptr<cilantro::PointCloud3f> single_cloud(new cilantro::PointCloud3f);
        for (int k = 0; k < labels.size(); k++)
        {
            if (i == labels[k])
            {
                // Push point
                single_cloud->points.conservativeResize(single_cloud->points.rows(), single_cloud->points.cols() + 1);
                single_cloud->points.col(single_cloud->points.cols() - 1) = ptt.col(k);

                // Push color
                if (color_point_cloud)
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col_seg.col(k);
                }
                if ((color_point_cloud == false) && cilantro_cloud_f->hasColors())
                {
                    single_cloud->colors.conservativeResize(single_cloud->colors.rows(), single_cloud->colors.cols() + 1);
                    single_cloud->colors.col(single_cloud->colors.cols() - 1) = col.col(k);
                }

                // Push normal
                if (cilantro_cloud_f->hasNormals())
                {
                    single_cloud->normals.conservativeResize(single_cloud->normals.rows(), single_cloud->normals.cols() + 1);
                    single_cloud->normals.col(single_cloud->normals.cols() - 1) = nor.col(k);
                }
            }
        }
        ptr_clouds_list.push_back(single_cloud);
    }

    return ptr_clouds_list;
};

///////////////////////////////////////////
///      RANSAC PLANE SEGMENTATION      ///
///////////////////////////////////////////

std::shared_ptr<open3d::geometry::PointCloud> planeSegmentation(std::shared_ptr <open3d::geometry::PointCloud> cloud, float distanceThreshold, int ransacN, int numIterations)
{
    std::tuple<Eigen::Vector4d, std::vector<size_t>> segmentation = cloud->SegmentPlane(distanceThreshold, ransacN, numIterations);
    Eigen::Vector4d plane = std::get<0>(segmentation);
    std::vector<size_t> inliersIndex = std::get<1>(segmentation);
    return cloud->SelectByIndex(inliersIndex, true); // Select out of plane points
}