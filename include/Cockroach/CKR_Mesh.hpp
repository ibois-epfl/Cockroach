#pragma once

#include <iostream>

// Open3D
#include "open3d/Open3D.h"
#include <open3d/pipelines/registration/RobustKernel.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>

typedef open3d::geometry::PointCloud PC;
typedef std::shared_ptr<PC> PC_ptr;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                       CLUSTER TECHNIQUES                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
Cockroach does not implement custom meshing techniques but instead uses the ones provided
by Open3D library, mainly Poission and Ball-pivoting reconstruction.

Source:
http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html
*/
