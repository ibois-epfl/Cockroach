#include <Cockroach.hpp>
#include <iostream>

int main()
{
    std::shared_ptr<open3d::geometry:PointCloud> cloud = importCloud("../cloud/cube.ply")
    Cockroach::visualize_standard(cloud)

    return 0;
}

