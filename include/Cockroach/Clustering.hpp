#pragma once

#include <iostream>

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

// Shorthands
typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
typedef std::shared_ptr<PC> PC_ptr;

namespace Cockroach
{
	/// \summary HCS cluster: method based on normals and radius search.
	/// \param voxel_size_search Size of voxel for point search, the smaller the more/smaller clusters it will produce.
	/// \param normal_threshold_degree The threshold of tolerance for normal evaluation in degree angle.
	/// \param min_cluster_size Minimum number of points in a cluster.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		float voxel_size_search = 0.1f,
		double normal_threshold_degree = 2.0,
		int min_cluster_size = 100,
		bool color_point_cloud = true)
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

	/// \summary HCS cluster : method based on normals and neighbour search.
	/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
	/// \param normal_threshold_degree The threshold of tolerance for normal evaluation in degree angle.
	/// \param min_cluster_size Minimum number of points in a cluster.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_connectedComponentKSearch(std::shared_ptr<cilantro::PointCloud3f>& cilantro_cloud_f,
		int n_search = 30,
		double normal_threshold_degree = 2,
		int min_cluster_size = 100,
		bool color_point_cloud = true)
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
		//return std::vector<std::shared_ptr<cilantro::PointCloud3f>>{cloud_seg};
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

	/// \summary Spectral cluster : method based on Laplacian function [description to be compelted].
	/// \param max_num_clusters Minimum number of clusters to be created.
	/// \param k_search Number of neighbours to be searched.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_spectralCluster(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		size_t max_num_clusters = 5,
		int k_search = 30,
		bool color_point_cloud = true)
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

	/// \summary Euclidean cluster : method based on HCS cluster with distance function as main descriptor.
	/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
	/// \param dist_thresh The distance threshold of points to be cluster together.
	/// \param min_cluster_size Minimum number of points in a cluster.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanKsearch(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		int n_search = 30,
		float dist_thresh = 0.02,
		int min_cluster_size = 30,
		bool color_point_cloud = true)
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

	/// \summary Euclidean cluster : method based on HCS cluster with distance function as main descriptor.
	/// \param voxel_size_search Size of voxel for point search, the smaller the more/smaller clusters it will produce.
	/// \param dist_thresh The distance threshold of points to be cluster together.
	/// \param min_cluster_size Minimum number of points in a cluster.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_euclideanRadius(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		float voxel_size_search,
		float dist_thresh,
		int min_cluster_size,
		bool color_point_cloud = true)
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

	/// \summary K-means cluster : iteratively finds spherical clusters of points (usually high-dimensional), where cluster affinity is based on a distance to the cluster center.
	/// \param n_cluster Number of cluster to cast.
	/// \param max_iter Maximal iterations.
	/// \param use_k_tree Use or not the k-tree search
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_kMeans(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		size_t n_cluster = 250,
		size_t max_iter = 100,
		bool use_k_tree = true,
		bool color_point_cloud = true)
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
	};

	/// \summary Mean shift clustering aims to discover �blobs� in a smooth density of samples.
	/// \param kernel_radius Radius of the kernel search.
	/// \param max_iter Max number of iterations.
	/// \param cluster_tol Tolerance of the clustering.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_meanShift(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		float kernel_radius = 2.0f,
		int max_iter = 5000,
		float cluster_tol = 0.2f,
		bool color_point_cloud = true)
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

	/// \summary Cluster points with the same color.
	/// \param n_search Number of neighbours to be included in the k-search, the more the better, but also the more expensive.
	/// \param dist_thresh The color distance threshold of points to be cluster together.
	/// \param min_cluster_size Minimum number of points in a cluster.
	/// \param color_point_cloud Apply random color to new cluster or leave original colors.
	inline std::vector<std::shared_ptr<cilantro::PointCloud3f>> cluster_color(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f,
		int n_search = 30,
		float dist_tresh = 0.01f,
		int min_cluster_size = 30,
		bool color_point_cloud = true)
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

	/// \summary Segment the biggest plane in the scene and retrieve the outlier points.
	/// \param distanceThreshold The istance threshold from the plane to include close points.
	/// \param numIterations Max number of iterations.
	inline std::shared_ptr<open3d::geometry::PointCloud> planeSegmentation(std::shared_ptr <open3d::geometry::PointCloud> cloud,
		float distanceThreshold = 0.01f,
		int ransacN = 3,
		int numIterations = 1000)
	{
		std::tuple<Eigen::Vector4d, std::vector<size_t>> segmentation = cloud->SegmentPlane(distanceThreshold, ransacN, numIterations);
		Eigen::Vector4d plane = std::get<0>(segmentation);
		std::vector<size_t> inliersIndex = std::get<1>(segmentation);
		return cloud->SelectByIndex(inliersIndex, true); // Select out of plane points
	};
}