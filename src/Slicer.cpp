#include "Slicer.hpp"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/aff_transformation_tags.h>
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

Clipper2Lib::Paths64 MetalSlicer::joinSegments(const std::vector<Kernel::Segment_3>& segments, double scale) {
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

Clipper2Lib::Paths64 MetalSlicer::sliceLayer(double z_height, double scale) {
    // 1. Define o plano horizontal (Z = z_height)
    // O vetor (0,0,1) indica que o plano é perpendicular ao eixo Z
    Kernel::Plane_3 plane(0, 0, 1, -z_height);

    // 2. Vetor para armazenar os resultados da intersecção
    std::vector<Tree::Intersection_and_primitive_id<Kernel::Plane_3>::Type> intersections;
    
    // 3. Intersecção eficiente usando a AABB Tree
    tree.all_intersections(plane, std::back_inserter(intersections));

    // 4. Filtra apenas os segmentos de reta encontrados
    std::vector<Kernel::Segment_3> segments;
    for (const auto& inter : intersections) {
        // O resultado da intersecção está no first (um boost::variant)
        // Tentamos extrair um segmento dele
        if (const Kernel::Segment_3* s = boost::get<Kernel::Segment_3>(&(inter.first))) {
            segments.push_back(*s);
        }
    }

    return joinSegments(segments, scale);
}

Clipper2Lib::Paths64 MetalSlicer::generateConcentricInfill(
    const Clipper2Lib::Paths64& boundary, 
    double bead_width,   // Largura do cordão (ex: 5.0mm)
    double overlap_pct,  // Sobreposição (ex: 0.7 para 70%)
    double oversize,     // Material extra para usinagem (ex: 1.5mm)
    double scale) 
{
    // Teste 1: O contorno original tem pontos?
    std::cout << "DEBUG: Pontos no contorno original: " << boundary[0].size() << std::endl;

    Paths64 current_layer;
    
    // PASSO 1: Aplicar Sobremetal (Oversizing)
    // Isso expande as paredes externas e contrai as internas (buracos)
    current_layer = InflatePaths(boundary, oversize * scale, 
                                 JoinType::Miter, EndType::Polygon);

      
    Paths64 all_trajectories;
    Paths64 last_offset = current_layer;
    
    // Adicionamos o primeiro contorno (já com sobremetal)
    all_trajectories.insert(all_trajectories.end(), last_offset.begin(), last_offset.end());

    // PASSO 2: Gerar anéis concêntricos
    // O passo do offset considera a largura do cordão e a sobreposição experimental
        // Teste 2: O oversize destruiu o polígono?
    if (current_layer.empty()) {
        std::cout << "DEBUG: Polígono desapareceu após o Oversize!" << std::endl;
        return {};
    }
    double step = -bead_width * (1.0 - overlap_pct) * scale;
    std::cout << "DEBUG: Step calculado: " << step << std::endl;

    while (true) {
        last_offset = InflatePaths(last_offset, step, JoinType::Miter, EndType::Polygon);
        
        if (last_offset.empty()) break; // Preenchimento completo
        
        all_trajectories.insert(all_trajectories.end(), last_offset.begin(), last_offset.end());
    }

    return all_trajectories;
}

CGAL::Bbox_3 MetalSlicer::getMeshBoundingBox() const {
    // Para Surface_mesh, a forma mais segura é usar o namespace PMP (Polygon Mesh Processing)
    // ou iterar sobre os pontos, mas o CGAL oferece este atalho:
    return CGAL::Polygon_mesh_processing::bbox(mesh);
}

size_t MetalSlicer::getFaceCount() const {
    return num_faces(mesh);
}