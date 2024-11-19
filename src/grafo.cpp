#include <stdexcept>

#include "./../include/grafo.hpp"

Vertice::Vertice() : _gr_entrada(0), _gr_saida(0) {}

void Vertice::add_gr_entrada(size_t n)
{
    _gr_entrada += n;
}

void Vertice::add_gr_saida(size_t n)
{
    _gr_saida += n;
}

size_t Vertice::get_gr_entrada() const
{
    return _gr_entrada;
}

size_t Vertice::get_gr_saida() const
{
    return _gr_saida;
}

void Vertice::add_adjacencia(size_t v)
{
    _adjacencias.push_back(v);
}

std::vector<size_t> &Vertice::adjacencias()
{
    return _adjacencias;
}

// -------

Grafo::Grafo() : _n_vertices(0) {}

Grafo::Grafo(size_t n_vertices) : _n_vertices(n_vertices)
{
    _vertices.assign(_n_vertices, Vertice());
}

size_t Grafo::add_vertice()
{
    _vertices.push_back(Vertice());
    _n_vertices++;

    // retorna o id do vértice inserido
    return _n_vertices - 1;
}

void Grafo::add_aresta(size_t v1, size_t v2)
{
    _vertices[v1].add_adjacencia(v2);
    _vertices[v1].add_gr_saida();
    _vertices[v2].add_gr_entrada();
}

size_t Grafo::tamanho() const
{
    return _n_vertices;
}

Vertice &Grafo::vertice(size_t u)
{
    return _vertices[u];
}

std::vector<Vertice> &Grafo::vertices()
{
    return _vertices;
}

Grafo Grafo::transposto()
{
    Grafo grafo = Grafo(_n_vertices);

    for (size_t u = 0; u != _n_vertices; u++)
    {
        auto &arestas = _vertices[u].adjacencias();
        for (const auto &v : arestas)
        {
            grafo.add_aresta(v, u);
        }
    }

    return grafo;
}

Subgrafo Grafo::subgrafo(std::vector<size_t> &subvertices)
{
    std::vector<size_t> mapa_grafo_subgrafo(_n_vertices);
    std::vector<char> usados(_n_vertices, 0);

    size_t i = 0;

    // cria mapeamento de id do grafo original para id do subgrafo
    // inicializa os ids na ordem dos vértices fornecidos
    for (const auto &v : subvertices)
    {
        mapa_grafo_subgrafo[v] = i;
        i++;
        usados[v] = 1;
    }

    Subgrafo subgrafo = Subgrafo(subvertices.size());

    for (const auto &u : subvertices)
    {
        auto &arestas = _vertices[u].adjacencias();
        size_t u_s = mapa_grafo_subgrafo[u];
        subgrafo.set_id_subgrafo_grafo(u_s, u);
        for (const auto &v : arestas)
        {
            size_t v_s = mapa_grafo_subgrafo[v];
            if (usados[v])
            {
                subgrafo.add_aresta(u_s, v_s);
                subgrafo.set_id_subgrafo_grafo(v_s, v);
            }
        }
    }
    return subgrafo;
}

// -------

Subgrafo::Subgrafo() : Grafo() {}

Subgrafo::Subgrafo(size_t n_vertices) : Grafo(n_vertices)
{
    _mapa_id_subgrafo_grafo.resize(n_vertices);
}

size_t Subgrafo::add_vertice()
{
    size_t s = Grafo::add_vertice();
    _mapa_id_subgrafo_grafo.push_back(0);

    return s;
}

void Subgrafo::set_id_subgrafo_grafo(size_t id_subgrafo, size_t id_grafo)
{
    _mapa_id_subgrafo_grafo[id_subgrafo] = id_grafo;
}

std::vector<size_t> &Subgrafo::mapa_id_subgrafo_grafo()
{
    return _mapa_id_subgrafo_grafo;
}