// #include <Cockroach.hpp>
#include <iostream>
#include <filesystem>
#include <Cockroach.hpp>


typedef open3d::geometry::PointCloud PC;
typedef std::shared_ptr<PC> PC_ptr;

string pathAppend(const string& p1, const string& p2) {

   char sep = '/';
   string tmp = p1;

#ifdef _WIN32
  sep = '\\';
#endif

  if (p1[p1.length()] != sep) { // Need to add a
     tmp += sep;                // path separator
     return(tmp + p2);
  }
  else
     return(p1 + p2);
}


int main()
{
    // Get current directory
    std::filesystem::path c_dir = std::filesystem::current_path().string();
    std::string p_dir = c_dir.parent_path().string();
    std::string cloud_name = p_dir + "cloud" + "/" + "cube.ply";

    std::shared_ptr<PC> cloud(new PC);
    cloud = Cockroach::importCloud(cloud_name);
    Cockroach::visualize_standard(cloud);

    // std::shared_ptr<PC> cloud(new PC);
    // const open3d::io::ReadPointCloudOption PC_options;
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


    return EXIT_SUCCESS;
}

