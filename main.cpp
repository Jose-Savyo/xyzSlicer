#include "src/Slicer.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: ./validator STLfiles/body.stl>" << std::endl;
        return 1;
    }

    MetalSlicer slicer;

    std::cout << "Carregando STL..." << std::endl;
    if (slicer.loadSTL(argv[1])) {
        std::cout << "Sucesso! Faces encontradas: " << slicer.getFaceCount() << std::endl;
        
        std::cout << "Construindo AABB Tree..." << std::endl;
        slicer.buildIndex();
        std::cout << "Fase 1 concluida com sucesso." << std::endl;
    }

    return 0;
}