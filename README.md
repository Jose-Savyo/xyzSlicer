# xyzSlicer - WAAM Geometry Engine 🛠️⚡

**xyzSlicer** is a specialized slicing engine developed for **Wire Arc Additive Manufacturing (WAAM)**. Unlike conventional plastic slicers, this engine is tailored for metal deposition, handling large bead widths and the specific requirement of "oversizing" for post-process machining.

---

## 🏗️ System Architecture

The project integrates high-precision 3D geometric processing with 2D trajectory manipulation:

- **Geometric Back-end:** Built with [CGAL](https://www.cgal.org/) (Computational Geometry Algorithms Library). It utilizes the **Exact Predicates Kernel** to eliminate rounding errors during complex intersections.
- **Trajectory Processing:** Integrated with [Clipper2](https://github.com/AngusJohnson/Clipper2) for polygon union and offsetting (inflating/deflating) operations at a micrometric scale using 64-bit integer arithmetic.



---

## 🚀 Key Features

### 1. Accelerated Slicing
- Employs **AABB Trees** (Axis-Aligned Bounding Boxes) for spatial indexing. This allows the intersection between the slicing plane and thousands of mesh triangles to occur in milliseconds.

### 2. Filling Strategy (Infill)
- **Concentric Infill:** Optimized to minimize weld arc ignitions, maintaining thermal continuity throughout the process.
- **Overlap Control:** Experimental adjustment of lateral overlap between beads to ensure 100% part density and prevent porosity.

### 3. Hybrid Manufacturing Focus
- **Oversizing (Machining Allowance):** Automatic generation of extra material on external and internal faces to ensure a sufficient margin for post-process milling/finishing.



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

# Compile the project
mkdir build && cd build
cmake ..
make

# Run with an STL file
./validator ../STLfiles/body.stl
