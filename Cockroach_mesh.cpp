#include <iostream>

// // For Rhino.rhp
// #include "StdAfx.h"


// Open3D
#include "open3d/Open3D.h"
#include <open3d/pipelines/registration/RobustKernel.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>

// Personal
#include "Cockroach.h"


typedef open3d::geometry::PointCloud PC;
typedef std::shared_ptr<PC> PC_ptr;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                       CLUSTER TECHNIQUES                                                                             ///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////
///       POISSON RECONSTRUCTION        ///
///////////////////////////////////////////

// Source:
/* http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html */
