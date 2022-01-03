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
///                                                                          UTILITIES                                                                                   ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////
///        COMPUTE PC THRESHOLD         ///
///////////////////////////////////////////

/* the "threshold" is a variable always used in registration of PCs. It's the maximal pair-distance between points of the two PCs.
 * In order to compute this value without hard-coding, we consider the average among all the distances between all point of source cloud to target cloud  */

double computeThreshold(std::shared_ptr<PC> cloudSource,
                       std::shared_ptr<PC> cloudTarget)
{
    std::vector<double> distances = cloudSource->ComputePointCloudDistance(*cloudTarget);
    //auto max_distance = *std::max_element(std::begin(distances), std::end(distances)); // MAX
    auto average_distance = std::accumulate(std::begin(distances), std::end(distances), 0.0) / distances.size(); // the average
    double distance_threshold = average_distance;
    return distance_threshold;
};


///////////////////////////////////////////
///       COMPUTE PC RESOLUTION         ///
///////////////////////////////////////////

/* TO BE IMRPOVED AND NOT TAKE IN ACCOUNT THE HOLES*/
/* The average distance from point to point of the same cloud */

float computePCResolution(std::shared_ptr <open3d::geometry::PointCloud> cloud)
{
    std::vector<double> pcdRes = cloud->ComputeNearestNeighborDistance();
    float average = 0.0f;
    if (pcdRes.size() != 0)
        average = std::accumulate(pcdRes.begin(), pcdRes.end(), 0.0) / pcdRes.size(); // [m]
    return average;
}


///////////////////////////////////////////
///             I/O_OPEN3D              ///
///////////////////////////////////////////

// import every type of cloud for Open3D files
std::shared_ptr<open3d::geometry::PointCloud> importCloud(std::string FILE_NAME_)
{
    std::shared_ptr<PC> cloud(new PC);
    const open3d::io::ReadPointCloudOption PC_options;
    open3d::io::ReadPointCloud(FILE_NAME_, *cloud, PC_options);
    if (cloud->HasPoints())
    {
        std::cout << "The point cloud has been correctly imported " << std::endl;
        std::cout << "Number of points " + std::to_string(cloud->points_.size())
            << std::endl;
        return cloud;
    }
    else
    {
        std::cout << "The point cloud has not been correctly imported, press "
            "any key to close"
            << std::endl;
        if (std::cin.get()) throw "No point cloud imported";
    }
};

// fetch directory
std::string getCurrentDirectory()
{
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    std::string::size_type pos = std::string(buffer).find_last_of("\\/");

    return std::string(buffer).substr(0, pos);
};


///////////////////////////////////////////
///          ESTIMATE NORMALS           ///
///////////////////////////////////////////

void estimateUnstructuredPCDNormals(std::shared_ptr<open3d::geometry::PointCloud> cloud, int numberOfCPUser, double radius_search, int iter)
{
    using namespace std;
    cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_search, iter), true);
    int numberOfCP = min(1000, max(10, (int)(numberOfCPUser)));
    cloud->OrientNormalsConsistentTangentPlane(numberOfCP);
};


///////////////////////////////////////////
///            DOWNSAMPLING             ///
///////////////////////////////////////////

std::shared_ptr<open3d::geometry::PointCloud> uniformDownsample(std::shared_ptr<open3d::geometry::PointCloud> cloud, int targetOfPoints)
{
    using namespace std;
    int nth = max(1, (int)(cloud->points_.size() * 1.0 / targetOfPoints * 1.00));
    cloud = cloud->UniformDownSample(nth);
    return cloud;
};

std::shared_ptr<open3d::geometry::PointCloud> voxelDownSampling(std::shared_ptr<open3d::geometry::PointCloud> cloud, float VoxelSize)
{
    // float VoxelSize = 0.05f = 5cm
    return cloud->VoxelDownSample(VoxelSize);
};


///////////////////////////////////////////
///           OUTLIER REMOVAL           ///
///////////////////////////////////////////

std::shared_ptr<open3d::geometry::PointCloud> statisticalOutlierRemoval(std::shared_ptr<open3d::geometry::PointCloud> cloud, int nbNeighbors, double stdRatio)
{
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>> statisticalOutlier = cloud->RemoveStatisticalOutliers(nbNeighbors, stdRatio);
    std::vector<size_t> filterIndexes = std::get<1>(statisticalOutlier);
    return std::get<0>(statisticalOutlier);
}


///////////////////////////////////////////
///                 OBB                 ///
///////////////////////////////////////////

std::shared_ptr<open3d::geometry::OrientedBoundingBox> computeOrientedBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> cloud)
{
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB(new open3d::geometry::OrientedBoundingBox(cloud->GetOrientedBoundingBox()));
    return OBB;
}


///////////////////////////////////////////
///            SUBSTRACT PC             ///
///////////////////////////////////////////

std::shared_ptr<open3d::geometry::PointCloud> substractCloud(std::shared_ptr<open3d::geometry::PointCloud> cloud_one, 
                                                             std::shared_ptr<open3d::geometry::PointCloud> cloud_two, 
                                                             double Ksearch_rad)
{

    // 1. Merge the clouds and keep track of cutting cloud indexes
    PC_ptr cloud_merge(new PC());
    *cloud_merge = *cloud_two + *cloud_one;
    int lastIndexCloudMerge = cloud_merge->points_.size() + 1;

    // 2. K-search
    std::vector<int> query_indexes_temp;
    std::set<int> query_indexes;
    std::vector<double> query_distances;
    open3d::geometry::KDTreeFlann Ksearch(*cloud_merge);

    for (int i = cloud_two->points_.size() + 1; i < cloud_merge->points_.size(); i++)
    {
        Ksearch.SearchRadius(cloud_merge->points_[i], Ksearch_rad, query_indexes_temp, query_distances);

        for (int id_hold : query_indexes_temp)
        {
            if (!(query_indexes.find(id_hold) != query_indexes.end()))
            {
                query_indexes.insert(i);
                query_indexes.insert(id_hold);
            }
        }
    }

    // 3. Cull the cloud with K-search indexes
    PC_ptr cloud_cull(new PC());
    for (int k = 0; k < cloud_merge->points_.size(); k++)
    {
        if (!(query_indexes.find(k) != query_indexes.end()))
        {
            Eigen::Vector3d pt_d = cloud_merge->points_[k];
            cloud_cull->points_.push_back(pt_d);

            if (cloud_merge->HasColors())
            {
                Eigen::Vector3d cl_d = cloud_merge->colors_[k];
                cloud_cull->colors_.push_back(cl_d);
            }

            if (cloud_merge->HasNormals())
            {
                Eigen::Vector3d no_d = cloud_merge->normals_[k];
                cloud_cull->normals_.push_back(no_d);
            }
        }
    }

    return cloud_cull;
}