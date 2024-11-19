#ifndef _ARVORES_
#define _ARVORES_

#include <vector>

#include "./../include/grafo.hpp"

// classe para armazenar os resultados de uma BFS
class ArvoreBFS
{
private:
    size_t _n_vertices;
    size_t _soma_distancias;

    // camadas simbolizadas por um vector
    // cada camada é simbolizada por um vector de pares (vértice, antecessor)
    std::vector<std::vector<std::pair<size_t, size_t>>> _camadas;

    // mapa para acesso aleatório, a fim de localizar em qual camada o vértice está
    // cada elemento é um par (camada, posição na camada)
    std::vector<std::pair<size_t, size_t>> _pos_vertice_camada;

public:
    // construtor que inicializa vértices e a soma de distâncias = 0
    ArvoreBFS();

    // método para adicionar primeira camada da BFS com o elemento "s"
    void add_primeira_camada(size_t s);

    // método para adicionar uma nova camada, após a última camada existente
    void add_camada();

    // método para adicionar um vértice e seu antecessor na camada da BFS
    void add_vertice_em_camada(size_t v, size_t ant, size_t camada);

    // método para remover a última camada adicionada
    void remover_ultima_camada();

    // método que retorna a quantidade de vértices explorados na BFS
    size_t n_vertices() const;

    // método que retorna a soma de distâncias ao percorrer na BFS
    size_t soma_distancias() const;

    // método que retorna uma referência para as camadas da BFS
    const std::vector<std::vector<std::pair<size_t, size_t>>> &camadas() const;

    // método que retorna uma referência para o mapeamento de vértices para as camadas
    const std::vector<std::pair<size_t, size_t>> &pos_vertice_camada() const;
};

// classe para armazenar os resultados de uma DFS
class ArvoreDFS
{
private:
    std::vector<size_t> _fechamentos;
    std::vector<Subgrafo> _componentes;

public:
    // método para adicionar uma componente gerada pela DFS
    void push_componente(Subgrafo &&subgrafo);

    // método para capturar a sequência de vértices
    // em ordem de fechamento ao realizar uma DFS
    void push_fechamentos(std::vector<size_t> &&fechamentos);

    // método para retornar uma referência para a ordem de fechamento
    const std::vector<size_t> &fechamentos() const;

    // método para retornar uma referência para as componentes geradas pela DFS
    std::vector<Subgrafo> &componentes();
};

#endif