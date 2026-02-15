#ifndef SLICER_HPP
#define SLICER_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <string>

// Definições de tipos do CGAL para clareza
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_3                                   Point_3;
typedef CGAL::Surface_mesh<Point_3>                       Mesh;

// Configuração da AABB Tree para faces da malha
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh>     Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive>              Traits;
typedef CGAL::AABB_tree<Traits>                           Tree;

class MetalSlicer {
public:
    MetalSlicer();

    // Carrega o arquivo STL para a memória
    bool loadSTL(const std::string& filename);

    // Constrói o índice espacial (AABB Tree)
    void buildIndex();

    // Apenas para verificação: retorna o número de faces da malha
    size_t getFaceCount() const;

private:
    Mesh mesh;
    Tree tree;
    bool is_indexed;
};

#endif