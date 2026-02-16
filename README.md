# xyzSlicer - WAAM Engine 🛠️⚡

xyzSlicer é um fatiador geométrico de alto desempenho desenvolvido para Manufatura Aditiva a Arco Elétrico (WAAM). O projeto foca na geração de trajetórias para robôs de deposição metálica, integrando algoritmos de geometria computacional para garantir precisão e robustez industrial.
🚀 Estado Atual do Projeto

O projeto encontra-se na Fase 2: Processamento Geométrico e Trajetórias. Atualmente, a engine é capaz de carregar malhas complexas, realizar intersecções em planos Z e processar polígonos para preenchimento concêntrico.
Funcionalidades Implementadas:

    AABB Tree Indexing: Utilização de árvores de intersecção aceleradas via CGAL para fatiamento de alto desempenho.

    Exact Predicates: Uso de kernels de aritmética exata para evitar erros de precisão em intersecções complexas.

    União de Polígonos: Conversão de segmentos de reta soltos em loops fechados via Clipper2.

    Oversizing (Sobremetal): Lógica integrada para adicionar material extra para processos de usinagem posterior.

    Preenchimento Concêntrico: Geração de anéis internos com controle experimental de overlap (sobreposição).

🏗️ Arquitetura Técnica

O projeto utiliza uma ponte de dados entre duas bibliotecas líderes de mercado:

    CGAL (Computational Geometry Algorithms Library): Responsável por toda a lógica 3D, carregamento de STL e intersecção de planos.

    Clipper2: Responsável pela manipulação 2D, offsets de contorno e operações booleanas de polígonos em escala micrométrica (utilizando aritmética de inteiros 64-bit).

🛠️ Requisitos e Compilação
Dependências

    CGAL 5.x+

    Clipper2

    Boost (especialmente boost::variant para intersecções)

    CMake e G++ (suporte a C++17 ou superior)

Como Compilar
Bash
