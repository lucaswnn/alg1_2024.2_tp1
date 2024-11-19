#include <limits>
#include <stack>
#include <stdexcept>
#include <unordered_map>

#include "./../include/algoritmos.hpp"

ArvoreBFS alg::BFS(Grafo &grafo, size_t s)
{
    if (s >= grafo.tamanho())
    {
        throw std::invalid_argument("O vértice s tem que estar no grafo");
    }

    ArvoreBFS arvore = ArvoreBFS();

    std::vector<char> descoberto(grafo.tamanho(), 0);
    descoberto[s] = 1;
    auto &camadas = arvore.camadas();
    arvore.add_primeira_camada(s);
    size_t camada = 0;

    while (!(camadas[camada].empty()))
    {
        arvore.add_camada();

        for (const auto &u_par : camadas[camada])
        {
            size_t u = u_par.first;
            for (const auto &v : grafo.vertice(u).adjacencias())
            {
                if (!descoberto[v])
                {
                    descoberto[v] = 1;
                    arvore.add_vertice_em_camada(v, u, camada + 1);
                }
            }
        }

        camada++;
    }

    // remove última camada vazia
    arvore.remover_ultima_camada();
    return arvore;
}

void alg::DFS_r(Grafo &grafo,
                std::vector<char> &descoberto,
                std::vector<size_t> &fechamento,
                size_t u)
{
    descoberto[u] = 1;

    for (const auto &v : grafo.vertice(u).adjacencias())
    {
        if (!descoberto[v])
        {
            DFS_r(grafo, descoberto, fechamento, v);
        }
    }

    fechamento.push_back(u);
}

void alg::DFS_r(Grafo &grafo,
                std::vector<char> &descoberto,
                std::vector<size_t> &fechamento,
                std::vector<size_t> &componentes,
                size_t u)
{
    descoberto[u] = 1;

    // coleta os vértices pertencentes a componente conexa gerada pela DFS
    componentes.push_back(u);

    for (const auto &v : grafo.vertice(u).adjacencias())
    {
        if (!descoberto[v])
        {
            DFS_r(grafo, descoberto, fechamento, componentes, v);
        }
    }

    fechamento.push_back(u);
}

// esta versão de DFS não se preocupa em capturar as componentes conexas geradas pela DFS
ArvoreDFS alg::DFS(Grafo &grafo)
{
    auto n = grafo.tamanho();
    std::vector<char> descoberto(n, 0);
    std::vector<size_t> fechamento;

    ArvoreDFS arvore;

    for (size_t u = 0; u != n; u++)
    {
        if (!descoberto[u])
        {
            DFS_r(grafo, descoberto, fechamento, u);
        }
    }

    arvore.push_fechamentos(std::move(fechamento));

    return arvore;
}

ArvoreDFS alg::DFS(Grafo &grafo, Grafo &grafo_t, std::vector<size_t> &ordem)
{
    auto n = grafo.tamanho();
    std::vector<char> descoberto(n, 0);
    std::vector<size_t> fechamento;

    // vector para coletar os vértices das componentes conexas geradas pela DFS
    std::vector<size_t> coleta_componentes;

    ArvoreDFS arvore;

    // executa a DFS na ordem fornecida
    for (auto u_it = ordem.begin(); u_it != ordem.end(); u_it++)
    {
        auto u = *u_it;
        if (!descoberto[u])
        {
            DFS_r(grafo_t, descoberto, fechamento, coleta_componentes, u);

            // adiciona a componente conexa na árvore da DFS completa
            arvore.push_componente(grafo.subgrafo(coleta_componentes));
            coleta_componentes.clear();
        }
    }

    arvore.push_fechamentos(std::move(fechamento));

    return arvore;
}

ArvoreDFS alg::SCC_Kosaraju(Grafo &grafo)
{
    // faz primeira DFS em uma ordem arbitrária
    auto dfs1 = alg::DFS(grafo);

    // a ordem definida é em relação aos tempos de fechamento
    // em ordem decrescente
    auto &fechamento1 = dfs1.fechamentos();
    std::vector<size_t> ordem = std::vector<size_t>(fechamento1.rbegin(), fechamento1.rend());

    auto grafo_t = grafo.transposto();
    auto dfs2 = alg::DFS(grafo, grafo_t, ordem);

    return dfs2;
}

// no pior caso, realiza BFS em todos os vértices para encontrar a menor soma de distâncias
ArvoreBFS alg::capital(Grafo &grafo)
{
    size_t n = grafo.tamanho();
    std::vector<char> candidatos(n, 1);

    // buffer para armazenar a menor soma de distâncias da BFS ao longo do algoritmo
    size_t menor_soma_distancias = std::numeric_limits<size_t>::max();

    ArvoreBFS bfs_ideal;

    for (size_t u = 0; u != n; u++)
    {
        if (!candidatos[u])
        {
            continue;
        }

        auto bfs = alg::BFS(grafo, u);
        auto n_vertices_percorridos = bfs.n_vertices();

        // se a BFS não percorreu todos os nós, não pode ser capital
        // e nem os vértices percorridos por transitividade
        // dessa forma o custo de realizar várias BFS é amortizado
        if (n_vertices_percorridos != n)
        {
            for (const auto &camada : bfs.camadas())
            {
                for (const auto &v_par : camada)
                {
                    size_t v = v_par.first;
                    candidatos[v] = 0;
                }
            }
            continue;
        }

        size_t soma_distancias = bfs.soma_distancias();
        if (soma_distancias < menor_soma_distancias)
        {
            menor_soma_distancias = soma_distancias;
            bfs_ideal = std::move(bfs);
        }
    }

    return bfs_ideal;
}

// para cada vértice de uma componente fortemente conexa, verifica a proximidade com a capital
// para cada candidato a batalhão, realiza-se uma BFS a partir de si
// e armazena a soma de distâncias percorridas
// em caso de empate na distância até a capital, ganha quem tiver a menor soma de distâncias
// no pior caso, realiza-se uma BFS para cada vértice da SCC
alg::Batalhoes_t alg::batalhoes(ArvoreDFS &componentes_dfs, ArvoreBFS &capital_bfs)
{
    auto &componentes = componentes_dfs.componentes();
    auto &mapa_bfs = capital_bfs.pos_vertice_camada();
    size_t u_capital = capital_bfs.camadas()[0][0].first;
    Batalhoes_t batalhoes;

    for (auto &scc : componentes)
    {
        bool contem_capital = false;
        auto &mapa_scc = scc.mapa_id_subgrafo_grafo();
        size_t menor_dist_capital = capital_bfs.n_vertices();
        size_t menor_dist_total = std::numeric_limits<size_t>::max();
        size_t batalhao_scc = 0;
        for (size_t u_scc = 0; u_scc != scc.tamanho(); u_scc++)
        {
            size_t u = mapa_scc[u_scc];
            if (u == u_capital)
            {
                contem_capital = true;
                batalhoes.push_back({0, u, u_scc});
                break;
            }

            size_t u_dist = mapa_bfs[u].first;
            if (u_dist < menor_dist_capital)
            {
                batalhao_scc = u_scc;
                menor_dist_capital = u_dist;
                auto bfs_candidato = BFS(scc, u_scc);

                // como a prioridade é a distância até a capital, sobrepõe-se a soma
                // de distâncias
                menor_dist_total = bfs_candidato.soma_distancias();
            }
            else if (u_dist == menor_dist_capital)
            {
                auto bfs_candidato = BFS(scc, u_scc);
                size_t dist_total_candidato = bfs_candidato.soma_distancias();
                if (dist_total_candidato < menor_dist_total)
                {
                    batalhao_scc = u_scc;
                    menor_dist_capital = u_dist;
                    menor_dist_total = dist_total_candidato;
                }
            }
        }

        // caso identificou-se que a SCC possui a capital, pula o loop
        if (contem_capital)
        {
            continue;
        }

        batalhoes.push_back({1, mapa_scc[batalhao_scc], batalhao_scc});
    }

    return batalhoes;
}

// esta versão de BFS adiciona novas arestas repetidas a um vértice com grau de entrada
// em excesso
std::pair<ArvoreBFS, alg::vec_add_arestas_t>
alg::BFS_patrulha(Subgrafo &subgrafo,
                  size_t s,
                  size_t s_exc_id,
                  std::vector<std::pair<size_t, alg::ll>> &excessos,
                  std::unordered_map<size_t, alg::ll> &faltas)
{
    if (s >= subgrafo.tamanho())
    {
        throw std::invalid_argument("O vértice s tem que estar no subgrafo");
    }

    ArvoreBFS arvore = ArvoreBFS();

    std::vector<char> descoberto(subgrafo.tamanho(), 0);
    descoberto[s] = 1;
    auto &camadas = arvore.camadas();
    arvore.add_primeira_camada(s);
    size_t camada = 0;

    size_t n_excessos = -excessos[s_exc_id].second;
    alg::vec_add_arestas_t vec_add_arestas;

    while (!(camadas[camada].empty()) && n_excessos != 0)
    {
        arvore.add_camada();

        for (const auto &u_par : camadas[camada])
        {
            size_t u = u_par.first;
            for (auto &v : subgrafo.vertice(u).adjacencias())
            {
                if (!descoberto[v])
                {
                    descoberto[v] = 1;
                    arvore.add_vertice_em_camada(v, u, camada + 1);

                    // durante a BFS, caso o vértice v tenha grau de saída em excesso,
                    // cria-se um caminho entre u e v até que um dos dois fique
                    // com grau equilibrado
                    // nota-se que dessa forma não é garantido que o algoritmo
                    // de hierholzer retorne um circuito euleriano mínimo
                    auto faltas_it = faltas.find(v);
                    if (faltas_it != faltas.end())
                    {
                        size_t n_faltas = faltas_it->second;
                        alg::ll add_n_arestas;
                        if (n_excessos > n_faltas)
                        {
                            add_n_arestas = n_excessos - n_faltas;
                            n_excessos -= n_faltas;
                            excessos[s_exc_id].second -= n_faltas;
                            faltas.erase(faltas_it);
                        }
                        else if (n_excessos < n_faltas)
                        {
                            add_n_arestas = n_faltas - n_excessos;
                            n_excessos = 0;
                            excessos[s_exc_id].second = 0;
                            faltas_it->second -= n_excessos;
                        }
                        else
                        {
                            add_n_arestas = n_excessos;
                            n_excessos = 0;
                            excessos[s_exc_id].second = 0;
                            faltas.erase(faltas_it);
                        }
                        vec_add_arestas.push_back(std::make_pair(s, std::make_pair(v, add_n_arestas)));
                    }
                }
            }
        }
        if (n_excessos == 0)
        {
            break;
        }

        camada++;
    }

    return std::make_pair(arvore, vec_add_arestas);
}

void alg::add_patrulhas_repetidas(ArvoreDFS &componentes)
{
    std::vector<std::pair<size_t, alg::ll>> excessos;
    std::unordered_map<size_t, alg::ll> faltas;

    for (auto &scc : componentes.componentes())
    {
        size_t s_scc = 0;
        // encontrar vértices desbalanceados
        for (const auto &vertice_scc : scc.vertices())
        {
            ll diff = (ll)vertice_scc.get_gr_saida() - (ll)vertice_scc.get_gr_entrada();
            if (diff < 0)
            {
                excessos.push_back(std::make_pair(s_scc, diff));
            }
            else if (diff > 0)
            {
                faltas.insert(std::make_pair(s_scc, diff));
            }
            s_scc++;
        }

        // adicionar arestas repetidas
        size_t s_count = 0;
        for (const auto &exc : excessos)
        {
            size_t s = exc.first;
            auto mapa_add_arestas = BFS_patrulha(scc, s, s_count, excessos, faltas);
            auto bfs = mapa_add_arestas.first;
            auto bfs_mapa = bfs.pos_vertice_camada();
            auto bfs_camadas = bfs.camadas();
            auto vec_add_arestas = mapa_add_arestas.second;
            for (const auto &add_aresta : vec_add_arestas)
            {
                size_t u = add_aresta.first;
                size_t v = add_aresta.second.first;
                size_t add_n_arestas = add_aresta.second.second;
                for (size_t i = 0; i != add_n_arestas; i++)
                {
                    // adiciona arestas ao longo do caminho entre u e v gerado pela BFS
                    size_t camada_v = bfs_mapa[v].first;
                    size_t pos_camada_v = bfs_mapa[v].second;
                    size_t v_ant = bfs_camadas[camada_v][pos_camada_v].second;
                    while (v_ant != u)
                    {
                        scc.add_aresta(v_ant, v);
                        v = v_ant;
                        v_ant = bfs_camadas[bfs_mapa[v_ant].first][bfs_mapa[v_ant].second].second;
                    }
                    scc.add_aresta(v_ant, v);
                }
            }
            s_count++;
        }

        excessos.clear();
        faltas.clear();
    }
}

std::vector<std::pair<size_t, size_t>> alg::hierholzer(Subgrafo &subgrafo, size_t origem)
{
    std::vector<size_t> cont_arestas;

    for (auto &v : subgrafo.vertices())
    {
        cont_arestas.push_back(v.adjacencias().size());
    }

    std::stack<size_t> caminho_corrente;

    std::vector<std::pair<size_t, size_t>> circuito;

    caminho_corrente.push(origem);
    size_t v_corrente = origem;
    auto &vertices = subgrafo.vertices();
    auto &mapa = subgrafo.mapa_id_subgrafo_grafo();

    while (!caminho_corrente.empty())
    {
        if (cont_arestas[v_corrente])
        {
            caminho_corrente.push(v_corrente);

            size_t prox_v = vertices[v_corrente].adjacencias().back();

            cont_arestas[v_corrente]--;
            vertices[v_corrente].adjacencias().pop_back();

            v_corrente = prox_v;
        }
        else
        {
            circuito.push_back(std::make_pair(mapa[v_corrente], v_corrente));
            v_corrente = caminho_corrente.top();
            caminho_corrente.pop();
        }
    }

    // retorna o circuito em ordem inversa, incluindo o vértice inicial no início e fim
    return circuito;
}

std::vector<std::vector<std::pair<size_t, size_t>>> alg::patrulhas(ArvoreDFS &componentes, Batalhoes_t batalhoes)
{
    auto &sccs = componentes.componentes();

    std::vector<std::vector<std::pair<size_t, size_t>>> patrulhas;
    size_t scc_id = 0;

    // para cada batalhão, verifica-se se é possível um caminho
    // caso seja possível, armazena-se o caminho
    for (auto &batalhao : batalhoes)
    {
        size_t u_scc_batalhao = batalhao[2];
        std::vector<std::pair<size_t, size_t>> circuito = hierholzer(sccs[scc_id], u_scc_batalhao);
        if (circuito.size() > 1)
        {
            patrulhas.push_back(std::move(circuito));
        }
        scc_id++;
    }

    return patrulhas;
}