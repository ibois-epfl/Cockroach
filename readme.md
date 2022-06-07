
**Cockroach**
 <p align="center">
  <img width="400" height="410" src="https://github.com/9and3/Cockroach/blob/Cockroach/Cockroach_logo.png">
</p>

<div align = "center">
    <a href = "https://app.travis-ci.com/github/kzampog/cilantro">
        <img src = "https://app.travis-ci.com/kzampog/cilantro.svg?branch=master" alt = "Build Status" />
    </a>
    <a href = "https://ibois-epfl.github.io/Cockroach-documentation/">
        <img src = "https://img.shields.io/badge/documentation-preliminary-orange" alt = "Documentation Status" />
    </a>
    <a href = "https://github.com/ibois-epfl/Cockroach">
        <img src = "https://img.shields.io/badge/platform-win-green" alt = "Platform" />
    </a>
    <a href = "https://github.com/ibois-epfl/Cockroach/blob/Cockroach/LICENSE">
        <img src = "https://img.shields.io/github/license/ibois-epfl/Cockroach" alt = "License" />
    </a>
</div>
<br/>


Cockroach is a plugin developed to introduce various commands for point cloud post-processing and meshing into Rhinoceros® [4]   environment based on reference functions already existing in the open-source library Open3D [1], CGAL [2], Cilantro [3].

Authors: Petras Vestartas, Andrea Settimi.

Supervision: Julien Gamerro.

The pointcloud processing tools focus on:
1.	fast and easy-to-use geometric manipulation, characterization and decomposition of point clouds directly in Rhinoceros6 and 7®.
2.	improving the  link between CAD modelling software (Rhinoceros®) and point-cloud processing.
3.	focus on the integration of point-cloud processing with other frameworks such as easy-to-use .NET programming languages (C#, IronPython, VB) using the interface of Grasshopper, Rhinoceros® [4].  

This plug-in is open-source to help researchers working for PointCloud processing in .NET.  At IBOIS - EPFL, among other research lines, we have also been focusing on structures with unpredictable geometries such as raw wood and mineral scraps. These construction elements are scanned and post-processed into low-poly meshes or NURBS for design, i.e. 3D timber joinery representation and fabrication tool-paths for 5-axis CNC, 6-axis robot and XR manufacture. 

We want to thank Dale Fugier (McNeel) for his help during C++ plugin development into Rhinoceros®.

[1] Zhou, Park, and Koltun, Open3D: A Modern Library for 3D Data Processing. http://www.open3d.org/

[2] CGAL, Computational Geometry Algorithms Library. https://www.cgal.org 

[3] Zampogiannis, Konstantinos and Fermuller, Cornelia and Aloimonos, Yiannis, Cilantro: A Lean, Versatile, and Efficient Library for Point Cloud Data Processing, Proceedings of the 26th ACM International Conference on Multimedia, 2018. https://github.com/kzampog/cilantro 

[4] https://www.rhino3d.com/ 

**Notes on the current version**

We are refactoring the library! This will take some time and this version represents a first milestone: a header-only library for windows. Next we will make it possible for Linux and refactor the code for Cockroach Rhino and Grasshopper.

**Dependencies**

Recent build and example files will be stored in Build directory. The method are run in two ways: a) Rhino plugin command-lines starting as Cockroach_CommandName, b) Grasshopper plugin. You will need to download the necessary dependencies to run the algorithms in Rhinoceros®. The dependencies could be acquired in two ways:
1.	Download Cockroach from www.food4rhino.com/
2.	Compile Open3D as a Dynamic Library from: Open3D repository: https://github.com/intel-isl/Open3D/blob/master/README.md. Compile Cilantro https://github.com/kzampog/cilantro and CGAL 5.2 https://github.com/CGAL/cgal/releases/tag/v5.2 .

**Citation**

Please use this citation if you use Cockroach in published work. Also, please also cite the third-party libraries we used: Open3D  (https://github.com/intel-isl/Open3D/blob/master/README.md) , CGAL (https://doc.cgal.org/latest/Manual/how_to_cite_cgal.html ), Cilantro (https://github.com/kzampog/cilantro/blob/master/README.md ).

Bibitex citation:
```bibitex
@misc{IBOIS2020, 
author = {Petras Vestartas and Andrea Settimi}, 
title = {{Cockroach}: {A} plug-in for point cloud post-processing and meshing in {Rhino} environment}, 
journal = {EPFL ENAC ICC IBOIS}, 
url = {https://github.com/9and3/Cockroach}, 
year = {2020} }
```

Citation (no Bibtex):
```
Petras Vestartas and Andrea Settimi, Cockroach: A Plug-in for Point Cloud Post-Processing and Meshing in Rhino Environment, EPFL ENAC ICC IBOIS, 2020, https://github.com/9and3/Cockroach.
```

**Contact**

For code request or chat, i.e. open GitHub issue or contact us by email. Some data, models or generated code using our research are available from the corresponding authors by request: [Petras Vestartas](petrasvestartas@gmail.com) or [Andrea Settimi](andrea.settimi@epfl.ch).

**Acknowledgements**

The laboratory for Timber Construction (IBOIS) at École Polytechnique Fédérale de Lausanne (EPFL) financially supports the authors contribution to the current research and version 1.0.0 development of the .NET PointCloud processing tool.

**License**

Cockroach is released under LGPL license. If you use Cockroach in published work, please also cite the third-party libraries we used: Open3D, CGAL, Cilantro. We encourage use for research purposes, as long as proper attribution is given. Feel free to send us an email and let us know how Cockroach has been useful to you and how it can be improved.

**How to Contribute?**

There are several ways how to use and contribute to Cockroach library:
a)	Compile C++ all or one of the 3rd party libraries (Open3D, CGAL, Cilantro) and use the project as it is. The libraries are divided into separate projects. Therefore, you can each project individually or together. The full project is called PInvoke. Individual C++ projects are PInvokeCGAL, PInvokeOpen3D, PInvokeCilantro.
b)	Use only .NET C# project – PinvokeCSharp, that wraps C++ libraries. Here you can write C# PointCloud processing algorithms too without 3rd party libraries.
c)	Use CockroachGH project to develop code for Rhino and Grasshopper.

If you have a valid contribution or bug fix, open an issue with the source code to merge it with the current project.

**Installation**

Visit our documentation for knowing more about which flavour to install of Cockroach.

[Here!!](https://ibois-epfl.github.io/Cockroach-documentation/)
