
#include <iostream>


#include <Cockroach.hpp>

typedef open3d::geometry::PointCloud PC;
typedef std::shared_ptr<PC> PC_ptr;




int main(int argc, char * argv[])
{
    // Open cloud
    std::shared_ptr<PC> cloud(new PC);
    cloud = Cockroach::importCloud("bug.ply");

    // // Convert Open3d>>Cilantro>>Open3d

    // // Visualize cloud
    Cockroach::visualize_standard(cloud);



    // std::cout << argv[0] << std::endl;
    // std::cout<<std::cin.get()<<std::endl;


    return EXIT_SUCCESS;
}

