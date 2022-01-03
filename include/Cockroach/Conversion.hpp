// #pragma once

// // Open3D
// #include "open3d/Open3D.h"
// #include <open3d/pipelines/registration/RobustKernel.h>
// #include <open3d/pipelines/registration/ColoredICP.h>
// #include <open3d/pipelines/registration/FastGlobalRegistration.h>

// // Cilantro
// #include <cilantro/utilities/point_cloud.hpp>

// //// PCL
// //#include <pcl/io/pcd_io.h>
// //#include <pcl/point_types.h>

// typedef open3d::geometry::PointCloud PC; // Opn3D point cloud
// typedef std::shared_ptr<PC> PC_ptr;


// namespace Cockroach 
// {
//     /// \summary Cilantro > OP3.
//     void convert_CilantroToOpen3DCloud(std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f, std::shared_ptr<PC> open3d_cloud_d)
//     {
//         auto ptt = cilantro_cloud_f->points;
//         int n_pt = (int)ptt.cols();
//         auto col = cilantro_cloud_f->colors;
//         auto nor = cilantro_cloud_f->normals;

//         for (int i = 0; i < n_pt; i++)
//         {
//             // Convert points
//             Eigen::Vector3d pt_d = ptt.col(i).cast<double>();
//             open3d_cloud_d->points_.push_back(pt_d);

//             // Convert colors
//             if (cilantro_cloud_f->hasColors())
//             {
//                 Eigen::Vector3d cl_d = col.col(i).cast <double> ();
//                 open3d_cloud_d->colors_.push_back(cl_d);
//             }

//             // Convert normals
//             if (cilantro_cloud_f->hasNormals())
//             {
//                 Eigen::Vector3d no_d = col.col(i).cast <double>();
//                 open3d_cloud_d->normals_.push_back(no_d);
//             }
//         }
//     };

//     /// \summary OP3 > Cilantro.
//     void convert_Open3DToCilantroCloud(std::shared_ptr<PC> open3d_cloud_d, std::shared_ptr<cilantro::PointCloud3f> cilantro_cloud_f)
//     {
//         // Convert points
//         auto ptt = cilantro_cloud_f->points;
//         for (auto& pt : open3d_cloud_d->points_)
//         {
//             Eigen::Vector3f pt_f = pt.cast <float>();
//             ptt.conservativeResize(ptt.rows(), ptt.cols() + 1);
//             ptt.col(ptt.cols() - 1) = pt_f;
//         }
//         cilantro_cloud_f->points = ptt;

//         // Convert colors
//         auto col = cilantro_cloud_f->colors;
//         if (open3d_cloud_d->HasColors())
//         {
//             for (auto& cl : open3d_cloud_d->colors_)
//             {
//                 Eigen::Vector3f cl_f = cl.cast <float>();
//                 col.conservativeResize(col.rows(), col.cols() + 1);
//                 col.col(col.cols() - 1) = cl_f;
//             }
//         }
//         cilantro_cloud_f->colors = col;


//         // Convert normals
//         auto nor = cilantro_cloud_f->normals;
//         if (open3d_cloud_d->HasNormals())
//         {
//             for (auto& nl : open3d_cloud_d->normals_)
//             {
//                 Eigen::Vector3f nl_f = nl.cast <float>();
//                 nor.conservativeResize(nor.rows(), nor.cols() + 1);
//                 nor.col(nor.cols() - 1) = nl_f;
//             }
//         }
//         cilantro_cloud_f->normals = nor;

//     };

//     // /// \summary PCLXYZ > OP3.
//     // void convert_PCLXYZToOpen3DCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud)
//     // {
//     //    const uint32_t size = pcl_cloud->size();
//     //    open3d_cloud->points_.resize(size);

//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        open3d_cloud->points_[i] = pcl_cloud->points[i].getVector3fMap().cast<double>();
//     //    }
//     // };

//     // /// \summary PCLNormal > OP3.
//     // void convert_PCLToOpen3DNormal(pcl::PointCloud<pcl::Normal>::Ptr pcl_normal, std::vector<Eigen::Vector3d> open3d_normal)
//     // {
//     //    const uint32_t size = pcl_normal->size();

//     //    open3d_normal.resize(size);

//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        open3d_normal[i] = pcl_normal->points[i].getNormalVector3fMap().cast<double>();
//     //    }

//     // };

//     // /// \summary PCLXYZRGB > OP3.
//     // void convert_PCLXYZRGBToOpen3DCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud)
//     // {
//     //    const uint32_t size = pcl_cloud->size();

//     //    open3d_cloud->points_.resize(size);
//     //    open3d_cloud->colors_.resize(size);

//     //    constexpr double normal = 1.0 / 255.0;
//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        open3d_cloud->points_[i] = pcl_cloud->points[i].getVector3fMap().cast<double>();
//     //        const uint32_t color = *reinterpret_cast<uint32_t*>(&pcl_cloud->points[i].rgb);
//     //        open3d_cloud->colors_[i] = Eigen::Vector3d((color >> 16) & 0x0000ff, (color >> 8) & 0x0000ff, color & 0x0000ff) * normal;
//     //    }
//     // };

//     // /// \summary PCLXYZRGBA > OP3.
//     // void convert_PCLXYZRGBAToOpen3Cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud)
//     // {
//     //    const uint32_t size = pcl_cloud->size();

//     //    open3d_cloud->points_.resize(size);
//     //    open3d_cloud->colors_.resize(size);

//     //    constexpr double normal = 1.0 / 255.0;
//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        open3d_cloud->points_[i] = pcl_cloud->points[i].getVector3fMap().cast<double>();
//     //        const uint32_t color = pcl_cloud->points[i].rgba;
//     //        open3d_cloud->colors_[i] = Eigen::Vector3d((color >> 16) & 0x000000ff, (color >> 8) & 0x000000ff, color & 0x000000ff) * normal;
//     //    }
//     // };

//     // /// \summary PCLXYZRGBNormal > OP3.
//     // void convert_PCLXYZRGBNormalToOpen3DCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud, std::shared_ptr<PC> open3d_cloud)
//     // {
//     //    const uint32_t size = pcl_cloud->size();

//     //    open3d_cloud->points_.resize(size);
//     //    open3d_cloud->normals_.resize(size);
//     //    open3d_cloud->colors_.resize(size);

//     //    constexpr double normal = 1.0 / 255.0;
//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        open3d_cloud->points_[i] = pcl_cloud->points[i].getVector3fMap().cast<double>();
//     //        open3d_cloud->normals_[i] = pcl_cloud->points[i].getNormalVector3fMap().cast<double>();
//     //        const uint32_t color = *reinterpret_cast<uint32_t*>(&pcl_cloud->points[i].rgb);
//     //        open3d_cloud->colors_[i] = Eigen::Vector3d((color >> 16) & 0x0000ff, (color >> 8) & 0x0000ff, color & 0x0000ff) * normal;
//     //    }
//     // };

//     // /// \summary OP3 > PCLXYZ.
//     // void convert_Open3DToPCLXYZCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
//     // {
//     //    const uint32_t size = open3d_cloud->points_.size();

//     //    pcl_cloud->width = size;
//     //    pcl_cloud->height = 1;
//     //    pcl_cloud->is_dense = false;
//     //    pcl_cloud->points.resize(size);

//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //    }
//     // };

//     // /// \summary OP3 > PCLNormal.
//     // void convert_Open3DToPCLNormal(std::vector<Eigen::Vector3d> open3d_normal, pcl::PointCloud<pcl::Normal>::Ptr pcl_normal)
//     // {
//     //    const uint32_t size = open3d_normal.size();
//     //    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        const Eigen::Vector3f normal = ( !open3d_normal.empty() ) ? open3d_normal[i].cast<float>() : zero;
//     //        std::copy(normal.data(), normal.data() + normal.size(), pcl_normal->points[i].normal);
//     //    }
//     // };

//     // /// \summary OP3 > PCLXYZRGB.
//     // void convert_Open3DToPCLXYZRGBCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud)
//     // {
//     //    const uint32_t size = open3d_cloud->points_.size();

//     //    pcl_cloud->width = size;
//     //    pcl_cloud->height = 1;
//     //    pcl_cloud->is_dense = false;
//     //    pcl_cloud->points.resize(size);

//     //    if (open3d_cloud->HasColors())
//     //    {
//     //        #pragma omp parallel for
//     //        for (int32_t i = 0; i < size; i++)
//     //        {
//     //            pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //            const auto color = (open3d_cloud->colors_[i] * 255.0).cast<uint32_t>();
//     //            uint32_t rgb = color[0] << 16 | color[1] << 8 | color[2];
//     //            pcl_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
//     //        }
//     //    }
//     //    else
//     //    {
//     //        #pragma omp parallel for
//     //        for (int32_t i = 0; i < size; i++)
//     //        {
//     //            pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //            uint32_t rgb = 0x000000;
//     //            pcl_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
//     //        }
//     //    }
//     // };

//     // /// \summary OP3 > PCLXYZRGBA.
//     // void convert_Open3DToPCLXYZRGBACloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud)
//     // {
//     //    const uint32_t size = open3d_cloud->points_.size();

//     //    pcl_cloud->width = size;
//     //    pcl_cloud->height = 1;
//     //    pcl_cloud->is_dense = false;
//     //    pcl_cloud->points.resize(size);

//     //    if (open3d_cloud->HasColors())
//     //    {
//     //        #pragma omp parallel for
//     //        for (int32_t i = 0; i < size; i++)
//     //        {
//     //            pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //            const auto color = (open3d_cloud->colors_[i] * 255.0).cast<uint32_t>();
//     //            pcl_cloud->points[i].rgba = 0xff000000 | color[0] << 16 | color[1] << 8 | color[2];
//     //        }
//     //    }
//     //    else
//     //    {
//     //        #pragma omp parallel for
//     //        for (int32_t i = 0; i < size; i++)
//     //        {
//     //            pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //            pcl_cloud->points[i].rgba = 0xff000000;
//     //        }
//     //    }
//     // };

//     // /// \summary OP3 > PCLXYZRGBNormal.
//     // void convert_Open3DToPCLXYZRGBNormalCloud(std::shared_ptr<PC> open3d_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud)
//     // {
//     //    const uint32_t size = open3d_cloud->points_.size();
//     //    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

//     //    pcl_cloud->width = size;
//     //    pcl_cloud->height = 1;
//     //    pcl_cloud->is_dense = false;
//     //    pcl_cloud->points.resize(size);

//     //    #pragma omp parallel for
//     //    for (int32_t i = 0; i < size; i++)
//     //    {
//     //        pcl_cloud->points[i].getVector3fMap() = open3d_cloud->points_[i].cast<float>();
//     //        const auto color = (open3d_cloud->colors_[i] * 255.0).cast<uint32_t>();
//     //        uint32_t rgb = (open3d_cloud->HasColors()) ? color[0] << 16 | color[1] << 8 | color[2] : 0x000000;
//     //        pcl_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
//     //        const Eigen::Vector3f normal = (open3d_cloud->HasNormals()) ? open3d_cloud->normals_[i].cast<float>() : zero;
//     //        std::copy(normal.data(), normal.data() + normal.size(), pcl_cloud->points[i].normal);
//     //    }
//     // };
// }