cmake_minimum_required(VERSION 3.15)
project(ValidationProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. CGAL e Dependências (Boost, GMP, MPFR)
find_package(CGAL REQUIRED COMPONENTS Core)

# 2. Clipper2 via FetchContent
include(FetchContent)
FetchContent_Declare(
    Clipper2
    GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
    GIT_TAG        main
)

# Faz o download e torna os caminhos disponíveis imediatamente
FetchContent_GetProperties(Clipper2)
if(NOT clipper2_POPULATED)
    FetchContent_Populate(Clipper2)
    # Adiciona o diretório da Clipper2 ao build
    add_subdirectory(${clipper2_SOURCE_DIR}/CPP ${clipper2_BINARY_DIR})
endif()

# 3. Executável
add_executable(validator main.cpp src/Slicer.cpp)

# 4. Configuração de Includes (O ponto crítico onde o erro ocorre)
# Aqui apontamos direto para onde o FetchContent baixou os arquivos
target_include_directories(validator PRIVATE src)

# 5. Linkagem
target_link_libraries(validator PRIVATE 
    CGAL::CGAL
    CGAL::CGAL_Core
    Clipper2
)

# Flags para evitar erros de arredondamento no CGAL
target_compile_options(validator PRIVATE -frounding-math)