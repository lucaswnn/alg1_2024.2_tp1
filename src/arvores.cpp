#include "./../include/arvores.hpp"

ArvoreBFS::ArvoreBFS() : _n_vertices(0), _soma_distancias(0) {};

void ArvoreBFS::add_primeira_camada(size_t s)
{
    _camadas.push_back({{s, s}});

    // aloca mais espaço  no mapa de posição dos vértices
    // para contemplar todos os vértices
    size_t n_aloc = s - _pos_vertice_camada.size() + 1;
    if (n_aloc > 0)
    {
        _pos_vertice_camada.insert(_pos_vertice_camada.end(), n_aloc, {0, 0});
    }
    _pos_vertice_camada[s] = {0, 0};

    _n_vertices++;
}

void ArvoreBFS::add_camada()
{
    _camadas.push_back({});
}

void ArvoreBFS::add_vertice_em_camada(size_t v, size_t ant, size_t camada)
{
    _camadas[camada].push_back({v, ant});
    typedef long long ll;

    // aloca mais espaço  no mapa de posição dos vértices
    // para contemplar todos os vértices
    ll n_aloc = (ll)v - (ll)_pos_vertice_camada.size() + 1;
    if (n_aloc > 0)
    {
        _pos_vertice_camada.insert(_pos_vertice_camada.end(), n_aloc, {0, 0});
    }
    _pos_vertice_camada[v] = {camada, _camadas[camada].size() - 1};

    _soma_distancias += camada;
    _n_vertices++;
}

void ArvoreBFS::remover_ultima_camada()
{
    _camadas.pop_back();
}

size_t ArvoreBFS::n_vertices() const
{
    return _n_vertices;
}

size_t ArvoreBFS::soma_distancias() const
{
    return _soma_distancias;
}

const std::vector<std::vector<std::pair<size_t, size_t>>> &ArvoreBFS::camadas() const
{
    return _camadas;
}

const std::vector<std::pair<size_t, size_t>> &ArvoreBFS::pos_vertice_camada() const
{
    return _pos_vertice_camada;
}

// -------

void ArvoreDFS::push_componente(Subgrafo &&subgrafo)
{
    _componentes.push_back(subgrafo);
}

void ArvoreDFS::push_fechamentos(std::vector<size_t> &&fechamentos)
{
    _fechamentos = fechamentos;
}

const std::vector<size_t> &ArvoreDFS::fechamentos() const
{
    return _fechamentos;
}

std::vector<Subgrafo> &ArvoreDFS::componentes()
{
    return _componentes;
}