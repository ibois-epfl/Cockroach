// #include <Cockroach.hpp>
#include <iostream>
#include <open3d/Open3D.h>

typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
typedef std::shared_ptr<PC> PC_ptr;


int main(int argc, char *argv[])
{
    // std::shared_ptr<open3d::geometry:PointCloud> cloud;
    // std::shared_ptr<open3d::geometry:PointCloud> cloud = importCloud("../cloud/cube.ply");
    // Cockroach::visualize_standard(cloud);

    std::shared_ptr<PC> cloud(new PC);
    const open3d::io::ReadPointCloudOption PC_options;
    // open3d::io::ReadPointCloud(FILE_NAME_, *cloud, PC_options);
    // if (cloud->HasPoints())
    // {
    //     std::cout << "The point cloud has been correctly imported " << std::endl;
    //     std::cout << "Number of points " + std::to_string(cloud->points_.size())
    //         << std::endl;
    //     return cloud;
    // }
    // else
    // {
    //     std::cout << "The point cloud has not been correctly imported, press "
    //         "any key to close"
    //         << std::endl;
    //     if (std::cin.get()) throw "No point cloud imported";
    // }

    std::cout << "Hello, world!" << std::endl;

    return 0;
}

