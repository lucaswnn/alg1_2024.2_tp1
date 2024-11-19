#include <iostream>
#include <unordered_map>
#include <string>

#include "./../include/algoritmos.hpp"
#include "./../include/grafo.hpp"

typedef std::unordered_map<size_t, std::string> mapa_ull_str_t;
typedef std::unordered_map<std::string, size_t> mapa_str_ull_t;

// processa o arquivo de entrada
std::pair<mapa_ull_str_t, Grafo> carregar_entrada()
{
    int n_vertices, n_arestas;
    std::cin >> n_vertices >> n_arestas;

    // associa cada centro urbano com um índice e vice-versa
    mapa_ull_str_t mapa_ull_str;
    mapa_str_ull_t mapa_str_ull;
    Grafo grafo = Grafo(n_vertices);

    // contador para diferenciar vértices e criar grafo ordenado pelos ids
    int contador = 0;

    for (int i = 0; i != n_arestas; i++)
    {
        std::string v1, v2;
        std::cin >> v1 >> v2;

        auto ins_v1 = mapa_str_ull.insert(std::make_pair(v1, contador));
        // se v1 ainda não existe no mapa
        if (ins_v1.second)
        {
            mapa_ull_str.insert(std::make_pair(contador, v1));
            contador++;
        }

        auto ins_v2 = mapa_str_ull.insert(std::make_pair(v2, contador));
        // se v2 ainda não existe no mapa
        if (ins_v2.second)
        {
            mapa_ull_str.insert(std::make_pair(contador, v2));
            contador++;
        }

        // pega os iteradores do mapa e insere o conteúdo no grafo
        grafo.add_aresta(ins_v1.first->second, ins_v2.first->second);
    }

    return std::make_pair(mapa_ull_str, grafo);
}

void imprimir_saida(mapa_ull_str_t &mapa,
                    ArvoreBFS &capital,
                    alg::Batalhoes_t &batalhoes,
                    std::vector<std::vector<std::pair<size_t, size_t>>> &patrulhas)
{
    size_t u_capital = capital.camadas()[0][0].first;
    std::cout << mapa.at(u_capital) << "\n";

    size_t n_batalhoes = batalhoes.size() - 1;
    std::cout << n_batalhoes << "\n";
    for (auto &batalhao : batalhoes)
    {
        if (batalhao[0] != 0)
        {
            std::cout << mapa.at(batalhao[1]) << "\n";
        }
    }

    size_t n_patrulhas = patrulhas.size();
    if (n_patrulhas == 0)
    {
        std::cout << 0 << std::endl;
    }
    else
    {
        std::cout << n_patrulhas << "\n";
    }
    for (size_t i = 0; i != n_patrulhas; i++)
    {
        auto patrulha = patrulhas[i];
        size_t tam_patrulha = patrulha.size();
        size_t u_centro;
        size_t j;
        for (j = tam_patrulha - 1; j != 1; j--)
        {
            u_centro = patrulha[j].first;
            std::cout << mapa.at(u_centro) << " ";
        }
        u_centro = patrulha[1].first;

        std::cout << mapa.at(u_centro) << std::endl;
    }
}

int main()
{
    auto entrada = carregar_entrada();
    auto mapa = entrada.first;
    auto grafo = entrada.second;

    auto capital = alg::capital(grafo);
    auto componentes = alg::SCC_Kosaraju(grafo);
    auto batalhoes = alg::batalhoes(componentes, capital);
    alg::add_patrulhas_repetidas(componentes);
    auto patrulhas = alg::patrulhas(componentes, batalhoes);
    imprimir_saida(mapa, capital, batalhoes, patrulhas);
}