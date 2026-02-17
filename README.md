# xyzSlicer - WAAM Geometry & Slicing Research 🛠️⚡

This project documents the development and research of slicing strategies for **Wire Arc Additive Manufacturing (WAAM)**. The goal was to create a robust toolpath generation engine capable of handling industrial metal deposition constraints.

---

## 📑 Project Evolution

### Phase 1: Python Prototype & ROS 2 Visualization (Completed) ✅
The initial stage focused on rapid prototyping and logic validation. Using Python 3, the project reached a functional stage of layer generation and path visualization.

* **Key Libraries:** * [Trimesh](https://trimesh.org/): For STL loading and mesh manipulation.
    * [Shapely](https://shapely.readthedocs.io/): For 2D polygon operations (`Polygon`, `LineString`).
    * [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/): Used for real-time toolpath visualization via **RViz2** using `MarkerArray`.
* **Accomplishments:** Successful implementation of Z-plane slicing, coordinate centering, and basic concentric infill.

**Installation (Phase 1):**
```bash
pip install trimesh shapely numpy
# Requires ROS 2 Jazzy installed on Ubuntu 24.04
```
### Phase 2: High-Performance C++ Engine (Completed) ✅
To reach industrial standards used by slicers like **Cura** and **PrusaSlicer**, the project migrated to a C++ architecture. This phase focused on computational efficiency and memory safety through the study of acceleration techniques like **AABB Trees**.

* **Key Libraries:**
    * [CGAL](https://www.cgal.org/): Used for advanced 3D geometry and AABB Trees to accelerate mesh intersections.
    * [Clipper2](http://www.angusj.com/clipper2/): Used for robust polygon offsetting and infill generation.
* **Accomplishments:** Implementation of accelerated slicing algorithms and a structure for "Oversizing" (machining allowance).

**Installation & Build (Phase 2):**
```bash
# 1. Install system dependencies
sudo apt install libcgal-dev libboost-all-dev

# 2. Clone this repository
git clone [https://github.com/seu-usuario/xyzSlicer.git](https://github.com/seu-usuario/xyzSlicer.git)
cd xyzSlicer

# 3. Build the engine
mkdir build && cd build
cmake ..
make

# 4. Run the validator
./validator ../STLfiles/body.stl

### 🏁 Conclusion & Practical Implementation (Phase 3)

After finalizing the C++ engine study and observing real-world WAAM experiments, a critical engineering conclusion was reached:

The current developed data and the experimental variables monitored in the field indicate that customizing an existing Cura Profile is the most efficient path for immediate practical application.

The insights gained from developing xyzSlicer (AABB Trees, Concentric Infill, and Bead Overlap) are now applied to fine-tune open-source slicers. This allows for total control over the variables required for the project's specific metal deposition study without the overhead of maintaining a custom geometry kernel.

### 📚 Research & References

This project was built upon the study of existing academic and technical solutions:

    slice_stl_create_path by Sunil Bhandari (MATLAB File Exchange).

    AMebius-slicer by Wang Jack (MATLAB File Exchange).

---
👨‍💻 Author

José Savyo - Electrical Engineering Student.
Focusing on Robotics, Automation, and Industrial Additive Manufacturing.
