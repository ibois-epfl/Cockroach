#include <iostream>

#include <Cockroach.hpp>



typedef open3d::geometry::PointCloud PC_o3d;
typedef std::shared_ptr<PC_o3d> PC_o3d_ptr;

typedef cilantro::PointCloud3f PC_cil;
typedef std::shared_ptr<PC_cil> PC_cil_ptr;

int main(int argc, char* argv[])
{
    // Open cloud
    PC_o3d_ptr cloud_op3_one(new PC_o3d);
    cloud_op3_one = Cockroach::importCloud("C:\\c++\\cockroach_1_0_0\\build\\Release\\bug.ply");


    //Compute normals
    Cockroach::estimateUnstructuredPCDNormals(cloud_op3_one, 16, 0.1);

    for (auto& normal : cloud_op3_one->normals_) {
        std::cout << normal << std::endl << std::endl;
    }


    // Convert Open3d>>Cilantro>>Open3d
    PC_cil_ptr cloud_cli(new PC_cil);
    Cockroach::convert_Open3DToCilantroCloud(cloud_op3_one, cloud_cli);
    PC_o3d_ptr cloud_op3_two(new PC_o3d);
    Cockroach::convert_CilantroToOpen3DCloud(cloud_cli, cloud_op3_two);

    // Visualize cloud
    Cockroach::visualize_standard(cloud_op3_two);

    return EXIT_SUCCESS;
}