#include "src/Slicer.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    double bead = 5.0;      
    double overlap = 0.25;  
    double extra = 1.5;     
    double scale = 1000000.0;

    if (argc < 2) {
        std::cout << "Uso: ./validator <caminho_do_stl>" << std::endl;
        return 1;
    }

    MetalSlicer slicer;

    std::cout << "Carregando STL: " << argv[1] << std::endl;
    
    // 1. Tenta carregar e indexar. Se falhar, encerra o programa com segurança.
    if (!slicer.loadSTL(argv[1])) {
        std::cerr << "ERRO: Falha ao carregar o arquivo!" << std::endl;
        return 1;
    }

    // ... após o loadSTL ...
    slicer.translateToOrigin(); 
    auto nova_bbox = slicer.getMeshBoundingBox();
    std::cout << "Nova BBox Z: " << nova_bbox.zmin() << " ate " << nova_bbox.zmax() << std::endl;

    double primeira_camada = 2.0; // 2mm de altura
    auto contornos = slicer.sliceLayer(primeira_camada, scale);

    if (contornos.empty()) {
        std::cout << "ERRO: Ainda sem contornos! Verificando uniao de segmentos..." << std::endl;
    } else {
        auto trajetorias = slicer.generateConcentricInfill(contornos, bead, overlap, extra, scale);
        std::cout << "Sucesso! Aneis gerados: " << trajetorias.size() << std::endl;
    }

    if (!slicer.getFaceCount() == 0) {
        auto bbox = slicer.getMeshBoundingBox(); // Você precisará criar esse getter no Slicer.cpp/hpp
        std::cout << "--- Dimensoes do STL ---" << std::endl;
        std::cout << "Z Min: " << bbox.zmin() << " | Z Max: " << bbox.zmax() << std::endl;
        std::cout << "X Range: " << bbox.xmin() << " a " << bbox.xmax() << std::endl;
        std::cout << "Y Range: " << bbox.ymin() << " a " << bbox.ymax() << std::endl;
        std::cout << "------------------------" << std::endl;
    }

    std::cout << "Sucesso! Faces: " << slicer.getFaceCount() << std::endl;
    slicer.buildIndex();
    std::cout << "Fase 1 concluida." << std::endl;
    
    auto bbox = slicer.getMeshBoundingBox(); 
    double z_teste = (bbox.zmin() + bbox.zmax()) / 2.0; 
    std::cout << "Testando fatiamento no meio da peca (Z = " << z_teste << ")..." << std::endl;
    // 2. Fatiamento (Z=10.0)
    auto contornos = slicer.sliceLayer(z_teste, scale); 

    // 3. Verificação crucial: se não houver contorno, não tente preencher
    if (contornos.empty()) {
        std::cout << "Aviso: Nenhum contorno encontrado em Z=10.0. A peca pode ser menor ou estar em outra posicao." << std::endl;
        return 0;
    }

    auto trajetorias = slicer.generateConcentricInfill(contornos, bead, overlap, extra, scale);
    std::cout << "Total de aneis gerados: " << trajetorias.size() << std::endl;

    return 0;
}