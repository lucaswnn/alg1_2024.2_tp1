#ifndef _ALGORITMOS_
#define _ALGORITMOS_

#include <vector>
#include <array>

#include "./../include/arvores.hpp"
#include "./../include/grafo.hpp"

// namespace com os algoritmos necessários para o problema
namespace alg
{
    typedef long long ll;

    // realiza BFS em um grafo, a partir de um vértice s
    ArvoreBFS BFS(Grafo &grafo, size_t s);

    // DFS base para recursão
    void DFS_r(Grafo &grafo,
               std::vector<char> &descoberto,
               std::vector<size_t> &fechamento,
               size_t u);

    // DFS base para recursão que recupera componentes
    void DFS_r(Grafo &grafo,
               std::vector<char> &descoberto,
               std::vector<size_t> &fechamento,
               std::vector<size_t> &componentes,
               size_t u);

    // retorna vértices em ordem de fechamento crescente da DFS e componentes conexas
    ArvoreDFS DFS(Grafo &grafo);

    // retorna vértices em ordem de fechamento crescente da DFS e componentes conexas
    // é possível fornecer a ordem de vértices para a execução da DFS
    ArvoreDFS DFS(Grafo &grafo, Grafo &grafo_t, std::vector<size_t> &ordem);

    // retorna as componentes fortemente conexas
    // algoritmo de Kosaraju
    ArvoreDFS SCC_Kosaraju(Grafo &grafo);

    // encontra a capital
    ArvoreBFS capital(Grafo &grafo);

    // Batalhoes_t composto de um vector de array[3]
    // cada elemento do vector representa um batalhão
    // array[0] simboliza se é uma capital (0) ou batalhão (1)
    // array[1] simboliza o id do vértice do grafo global
    // array[2] simboliza o id do vértice da componente fortemente conexa
    typedef std::vector<std::array<size_t, 3>> Batalhoes_t;

    // função para encontrar os batalhões dadas as componentes fortemente conexas
    // e a árvore BFS da capital
    Batalhoes_t batalhoes(ArvoreDFS &componentes_dfs, ArvoreBFS &capital_bfs);

    // vec_add_arestas_t simboliza um vector de arestas a serem adicionadas
    // nas componentes fortemente conexas para executar o algoritmo de hierholzer
    // cada elemento do vector possui um par (ponto inicial, (ponto final, n_arestas a adicionar))
    typedef std::vector<std::pair<size_t, std::pair<size_t, size_t>>> vec_add_arestas_t;

    // realiza BFS para encontrar vértices com graus desbalanceados
    std::pair<ArvoreBFS, vec_add_arestas_t>
    BFS_patrulha(Subgrafo &subgrafo,
                 size_t s,
                 size_t s_exc_id,
                 std::vector<std::pair<size_t, alg::ll>> &excessos,
                 std::unordered_map<size_t, alg::ll> &faltas);

    // função para adicionar arestas repetidas para obter um grafo passível de realizar
    // o algoritmo de hierholzer
    void add_patrulhas_repetidas(ArvoreDFS &componentes);

    // algoritmo de hierholzer para encontrar um ciclo euleriano
    // retorna um vector que simboliza o caminho na ordem reversa
    // cada elemento possui um par (id vértice global, id vértice componente)
    std::vector<std::pair<size_t, size_t>> hierholzer(Subgrafo &subgrafo, size_t origem);

    // função que encontra as possíveis patrulhas de um grafo
    std::vector<std::vector<std::pair<size_t, size_t>>>
    patrulhas(ArvoreDFS &componentes, Batalhoes_t batalhoes);
}

#endif