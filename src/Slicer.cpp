#include "Slicer.hpp"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
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

size_t MetalSlicer::getFaceCount() const {
    return num_faces(mesh);
}