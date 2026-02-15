#include "Slicer.hpp"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include "clipper2/clipper.h"
#include <iostream>

MetalSlicer::MetalSlicer() : is_indexed(false) {}

bool MetalSlicer::loadSTL(const std::string& filename) {
    mesh.clear();
    // Tenta ler o arquivo e armazenar na estrutura Surface_mesh
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(filename, mesh)) {
        std::cerr << "Erro: Nao foi possivel ler o arquivo " << filename << std::endl;
        return false;
    }
    return true;
}

void MetalSlicer::buildIndex() {
    if (mesh.is_empty()) return;

    // Insere todas as faces da malha na árvore
    tree.insert(faces(mesh).first, faces(mesh).second, mesh);
    
    // Constrói a estrutura de busca
    tree.build();
    is_indexed = true;
}

using namespace Clipper2Lib;

Paths64 joinSegments(const std::vector<Kernel::Segment_3>& segments, double scale) {
    Paths64 subjects;

    for (const auto& seg : segments) {
        Path64 p;
        
        // 1. Extrai o valor numérico exato do CGAL para double
        double x_src = CGAL::to_double(seg.source().x());
        double y_src = CGAL::to_double(seg.source().y());
        double x_tgt = CGAL::to_double(seg.target().x());
        double y_tgt = CGAL::to_double(seg.target().y());

        // 2. Multiplica pela escala e faz o cast manual para int64_t
        // Isso evita que o Clipper2 tente fazer um static_cast interno problemático
        p.push_back(Point64(static_cast<int64_t>(x_src * scale), 
                            static_cast<int64_t>(y_src * scale)));
                            
        p.push_back(Point64(static_cast<int64_t>(x_tgt * scale), 
                            static_cast<int64_t>(y_tgt * scale)));
        
        subjects.push_back(p);
    }

    // Unir segmentos adjacentes
    Paths64 joined = Union(subjects, FillRule::EvenOdd);
    // Uma tolerância de 0.02 precisa ser testada
    Paths64 simplified = SimplifyPaths(joined, 0.02 * scale);

    return simplified;
}

size_t MetalSlicer::getFaceCount() const {
    return num_faces(mesh);
}