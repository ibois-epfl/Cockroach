// #pragma once

// #include <iostream>

// // Open3D
// #include "open3d/Open3D.h"


// namespace Cockroach 
// {
//     /// \summary Standard visualization.
//     void visualize_standard(std::shared_ptr<open3d::geometry::PointCloud> cloud)
//     {
//         cloud->EstimateNormals();
//         std::shared_ptr<open3d::visualization::Visualizer> vis(new open3d::visualization::Visualizer());
//         vis->CreateVisualizerWindow("Open3DVis", 1000, 800, 500, 200);
//         vis->AddGeometry(cloud);
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(0, 0, 0);
//         vis->Run();
//         vis->DestroyVisualizerWindow();
//     };

//     /// \summary Visualize two clouds for registration.
//     void visualize_bicolor(std::shared_ptr<open3d::geometry::PointCloud> cloudSource, std::shared_ptr<open3d::geometry::PointCloud> cloudTarget, Eigen::Matrix4d transformation)
//     {
//         // Color palette
//         const int NUM_OF_COLOR_PALETTE = 5;
//         Eigen::Vector3d color_palette[NUM_OF_COLOR_PALETTE] = {
//                 Eigen::Vector3d(255, 180, 0) / 255.0,
//                 Eigen::Vector3d(0, 166, 237) / 255.0,
//                 Eigen::Vector3d(246, 81, 29) / 255.0,
//                 Eigen::Vector3d(127, 184, 0) / 255.0,
//                 Eigen::Vector3d(13, 44, 84) / 255.0,
//         };

//         // Target/Source PC
//         auto pcd_target = cloudTarget;
//         pcd_target->colors_.clear();
//         pcd_target->colors_.resize(pcd_target->points_.size(),
//                                 color_palette[0]);
//         auto pcd_source = cloudSource;
//         pcd_source->colors_.clear();
//         pcd_source->colors_.resize(pcd_source->points_.size(),
//                                 color_palette[1]);

//         // Apply transformation if there is one
//         if (transformation.size() != 0)
//             pcd_source->Transform(transformation);
//         else
//         {
//             // Visualize registered PCs
//             std::shared_ptr<open3d::visualization::Visualizer> vis(new open3d::visualization::Visualizer());
//             vis->CreateVisualizerWindow("RegistrationVisualize", 1500, 1500, 1000, 200);
//             vis->AddGeometry(pcd_source);
//             vis->AddGeometry(pcd_target);
//             vis->GetRenderOption().background_color_ = Eigen::Vector3d(255.00, 255.00, 255.00);
//             vis->GetRenderOption().mesh_color_option_ = open3d::visualization::RenderOption::MeshColorOption::Normal;
//             vis->GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::Normal;
//             vis->Run();
//             vis->DestroyVisualizerWindow();
//             return;
//         }
//         return;
//     };

//     void visualize_bicolor(std::shared_ptr<open3d::geometry::PointCloud> cloudSource, std::shared_ptr<open3d::geometry::PointCloud> cloudTarget)
//     {
//         // Color palette
//         const int NUM_OF_COLOR_PALETTE = 5;
//         Eigen::Vector3d color_palette[NUM_OF_COLOR_PALETTE] = {
//                 Eigen::Vector3d(255, 180, 0) / 255.0,
//                 Eigen::Vector3d(0, 166, 237) / 255.0,
//                 Eigen::Vector3d(246, 81, 29) / 255.0,
//                 Eigen::Vector3d(127, 184, 0) / 255.0,
//                 Eigen::Vector3d(13, 44, 84) / 255.0,
//         };

//         // Target/Source PC
//         auto pcd_target = cloudTarget;
//         pcd_target->colors_.clear();
//         pcd_target->colors_.resize(pcd_target->points_.size(),
//                                 color_palette[0]);
//         auto pcd_source = cloudSource;
//         pcd_source->colors_.clear();
//         pcd_source->colors_.resize(pcd_source->points_.size(),
//                                 color_palette[1]);

//         // Visualize registered PCs
//         std::shared_ptr<open3d::visualization::Visualizer> vis(new open3d::visualization::Visualizer());
//         vis->CreateVisualizerWindow("RegistrationVisualize", 1500, 1500, 1000, 200);
//         vis->AddGeometry(pcd_source);
//         vis->AddGeometry(pcd_target);
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(255.00, 255.00, 255.00);
//         vis->GetRenderOption().mesh_color_option_ = open3d::visualization::RenderOption::MeshColorOption::Normal;
//         vis->GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::Normal;
//         vis->Run();
//         vis->DestroyVisualizerWindow();

//         return;
//     };

//     /// \summary Visualize cloud with normals.
//     void visualize_cloudNormals(std::shared_ptr<open3d::geometry::PointCloud> cloud)
//     {
//         cloud->EstimateNormals();
//         std::shared_ptr<open3d::visualization::Visualizer> vis(new open3d::visualization::Visualizer());
//         vis->CreateVisualizerWindow("NormalVis", 1500, 1500, 1000, 200);
//         vis->AddGeometry(cloud);
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(255.00, 255.00, 255.00);
//         vis->GetRenderOption().mesh_color_option_ = open3d::visualization::RenderOption::MeshColorOption::Normal;
//         vis->GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::Normal;
//         vis->Run();
//         vis->DestroyVisualizerWindow();
//     };
// }
