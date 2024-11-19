#ifndef _GRAFO_
#define _GRAFO_

#include <cstddef>
#include <vector>

// classe que simboliza um vértice de um grafo
class Vertice
{
private:
    std::vector<size_t> _adjacencias;
    size_t _gr_entrada, _gr_saida;

public:
    // construtor que inicializa os graus = 0
    Vertice();

    // método para adicionar graus de entrada
    void add_gr_entrada(size_t n = 1);

    // método para adicionar graus de saída
    void add_gr_saida(size_t n = 1);

    // método que retorna o grau de entrada do vértice
    size_t get_gr_entrada() const;

    // método que retorna o grau de saída do  vértice
    size_t get_gr_saida() const;

    // método que adiciona uma nova aresta saindo do vértice
    void add_adjacencia(size_t v);

    // método que retorna uma referência para as adjacências do vértice
    std::vector<size_t> &adjacencias();
};

// declaração necessária para compilação
class Subgrafo;

// classe que simboliza um grafo por meio de lista de adjacências
class Grafo
{
protected:
    size_t _n_vertices;
    std::vector<Vertice> _vertices;

public:
    // construtor que inicializa um vértice vazio
    Grafo();

    // construtor que reserva um número inicial de vértices
    Grafo(size_t n_vertices);

    // método que adiciona um novo vértice disjunto
    virtual size_t add_vertice();

    // método que adiciona uma nova aresta entre dois vértices
    void add_aresta(size_t v1, size_t v2);

    // método que retorna número de vértices do grafo
    size_t tamanho() const;

    // método que retorna uma referência para determinado vértice
    Vertice &vertice(size_t u);

    // método que retorna uma referência para o vector de vértices
    std::vector<Vertice> &vertices();

    // método que produz um grafo transposto de si mesmo
    Grafo transposto();

    // método que produz um subgrafo a partir de uma lista de vértices
    // remove arestas entre vértices que não estão entre os vértices fornecidos
    Subgrafo subgrafo(std::vector<size_t> &subvertices);
};

// classe subgrafo derivada de grafo
class Subgrafo : public Grafo
{
private:
    // mapa que converte o id do vértice do subgrafo no id do vértice do grafo original
    std::vector<size_t> _mapa_id_subgrafo_grafo;

public:
    // construtor que ichama o construtor da superclasse
    Subgrafo();

    // construtor que chama o construtor da superclasse com um número inicial de vértices
    // e redimensiona o vector de mapa de ids de vértices
    Subgrafo(size_t n_vertices);

    // método que sobreescreve a superclasse, a fim de associar o vértice do subgrafo
    // ao grafo original, alterando o mapa de ids de vértices
    size_t add_vertice() override;

    // método que cria o mapeamento entre id do vértice do subgrafo e o grafo original
    void set_id_subgrafo_grafo(size_t id_subgrafo, size_t id_grafo);

    // método que retorna uma referência para o mapa de ids de vértices
    std::vector<size_t> &mapa_id_subgrafo_grafo();
};

#endif