# xyzSlicer - WAAM Geometry Engine 🛠️⚡

Unlike conventional plastic (FDM) slicers, this engine is engineered for metal deposition, managing large bead widths and the necessary "oversizing" for post-process machining.

---

## 📚 Research & Foundations

The development of this engine is grounded in a technical study of state-of-the-art slicing algorithms. The core logic for mesh intersection and toolpath sequencing was influenced by benchmarking two prominent implementations from the **MATLAB File Exchange**:

1.  **slice_stl_create_path** (v4.0.30) by **Sunil Bhandari**: Advanced study on robust STL slicing and path creation techniques.
2.  **AMebius-slicer** (v0.2.0) by **Wang Jack**: Analysis of STL slicing and G-code generation strategies for 3D printing environments.

By porting and optimizing these concepts into a high-performance **C++ (CGAL/Clipper2)** environment, **xyzSlicer** achieves industrial-grade execution speeds, memory safety, and precision.

---

## 🏗️ System Architecture

The project bridges 3D geometric processing with 2D trajectory optimization:

- **Geometric Back-end:** Powered by [CGAL](https://www.cgal.org/) (Computational Geometry Algorithms Library). It uses an **Exact Predicates Kernel** to eliminate rounding errors during plane-to-mesh intersections.
- **Trajectory Processing:** Integrated with [Clipper2](https://github.com/AngusJohnson/Clipper2) for polygon union, simplification, and offsetting (inflating/deflating) using 64-bit integer arithmetic.

🔍 Diagnostics and Telemetry

Built-in tools to validate geometry before processing:

    Bounding Box Calculation: Real-time detection of XYZ limits and part orientation.

    Path Closure Verification: Ensures all segments form manifold loops before toolpath generation.

    Memory Safety: Robust handling of empty intersections to prevent runtime segmentation faults.

🛤️ Development Roadmap

    [x] Phase 1: 3D Indexing - Implementation of AABB Tree via CGAL for fast spatial queries.

    [x] Phase 2: 2D Geometry - Integration with Clipper2 for robust polygon offsetting and union.

    [ ] [CURRENT] Intersection Refactoring - Debugging Z-plane intersection filters to resolve 0-contour detection issues.

    [ ] Coordinate Alignment - Implement manual and automatic mesh translation to the build plate (Z=0).

    [ ] G-Code Generation - Develop the export class with integrated Probe Scripts for automated height zeroing between layers.

    [ ] CLI Parameters - Support for experimental parameters (bead width, overlap) via command line or config files.


---

## 🚀 Key Features

### 1. Accelerated Slicing
Utilizes **AABB Trees** (Axis-Aligned Bounding Boxes) for spatial indexing. This enables the engine to query intersections across thousands of mesh triangles in milliseconds, ensuring scalability for complex industrial parts.

### 2. Filling Strategy (Infill)
- **Concentric Infill:** Specifically optimized for WAAM to minimize arc ignitions and extinctions, which are critical points for weld defects and heat accumulation.
- **Overlap Control:** Experimental adjustment of lateral overlap between beads to ensure 100% density and structural integrity.

### 3. Machining Allowance (Oversizing)
Automatic generation of extra material on both external and internal (holes) faces. This ensures sufficient "meat" remains on the part for post-process CNC milling to reach final tolerances.



---

## 🛠️ Build and Execution

### Prerequisites
* **Compiler:** GCC 11+ or Clang (C++17 support)
* **Libraries:** CGAL, Clipper2, Boost
* **Build System:** CMake

### Installation
```bash
# Clone the repository
git clone [https://github.com/your-username/xyzSlicer.git](https://github.com/your-username/xyzSlicer.git)
cd xyzSlicer

# Compile
mkdir build && cd build
cmake ..
make

# Run with an STL file
./validator ../STLfiles/body.stl
``` 
---
👨‍💻 Author

José Savyo - Electrical Engineering Student focusing on Robotics and Automation.

Experienced in hardware and software integration for industrial additive manufacturing.
