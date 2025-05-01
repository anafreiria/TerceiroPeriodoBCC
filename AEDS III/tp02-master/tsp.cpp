#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>

using namespace std;

struct City {
    int id;
    double x, y;
};

vector<City> cities;
vector<vector<double>> dist;

// Lê arquivo TSP e calcula a matriz de distâncias
void readTSPFile(const string& filename) {
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        if (line.find("NODE_COORD_SECTION") != string::npos) break;
    }
    City city;
    while (file >> city.id >> city.x >> city.y) {
        cities.push_back(city);
    }
    int n = cities.size();
    dist.resize(n, vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double dx = cities[i].x - cities[j].x;
            double dy = cities[i].y - cities[j].y;
            dist[i][j] = sqrt(dx*dx + dy*dy);
        }
    }
}

// Força Bruta (fixa a primeira cidade e permuta as restantes)
double bruteForceTSP(vector<int>& bestPath) {
    int n = cities.size();
    if (n <= 1) return 0;
    vector<int> path(n);
    for (int i = 0; i < n; ++i) path[i] = i;
    double minCost = numeric_limits<double>::max();
    do {
        double cost = 0;
        for (int i = 0; i < n; ++i) {
            cost += dist[path[i]][path[(i+1)%n]];
        }
        if (cost < minCost) {
            minCost = cost;
            bestPath = path;
        }
    } while (next_permutation(path.begin() + 1, path.end()));
    return minCost;
}

// Algoritmo Guloso (Vizinho Mais Próximo)
double greedyTSP(vector<int>& bestPath) {
    int n = cities.size();
    double minTotal = numeric_limits<double>::max();
    for (int start = 0; start < n; ++start) {
        vector<bool> visited(n, false);
        vector<int> path = {start};
        visited[start] = true;
        double total = 0;
        int current = start;
        for (int i = 0; i < n-1; ++i) {
            int next = -1;
            double minDist = numeric_limits<double>::max();
            for (int j = 0; j < n; ++j) {
                if (!visited[j] && dist[current][j] < minDist) {
                    minDist = dist[current][j];
                    next = j;
                }
            }
            path.push_back(next);
            visited[next] = true;
            total += minDist;
            current = next;
        }
        total += dist[current][start];
        if (total < minTotal) {
            minTotal = total;
            bestPath = path;
        }
    }
    return minTotal;
}

// Programação Dinâmica (Held-Karp)
double dpTSP(vector<int>& bestPath) {
    int n = cities.size();
    if (n > 20) return -1;
    vector<vector<double>> memo(1 << n, vector<double>(n, -1));
    vector<vector<int>> parent(1 << n, vector<int>(n, -1));
    memo[1][0] = 0; // Base: start na cidade 0

    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int last = 0; last < n; ++last) {
            if ((mask & (1 << last)) && memo[mask][last] != -1) {
                for (int next = 0; next < n; ++next) {
                    if (!(mask & (1 << next))) {
                        int newMask = mask | (1 << next);
                        double newCost = memo[mask][last] + dist[last][next];
                        if (memo[newMask][next] == -1 || newCost < memo[newMask][next]) {
                            memo[newMask][next] = newCost;
                            parent[newMask][next] = last;
                        }
                    }
                }
            }
        }
    }

    // Encontra o menor custo
    double minCost = numeric_limits<double>::max();
    int lastCity = -1;
    int fullMask = (1 << n) - 1;
    for (int i = 0; i < n; ++i) {
        if (i == 0) continue;
        if (memo[fullMask][i] != -1) {
            double total = memo[fullMask][i] + dist[i][0];
            if (total < minCost) {
                minCost = total;
                lastCity = i;
            }
        }
    }

    // Reconstrói o caminho
    bestPath.clear();
    int current = lastCity;
    int mask = fullMask;
    while (current != 0) {
        bestPath.push_back(current);
        int prev = parent[mask][current];
        mask ^= (1 << current);
        current = prev;
    }
    bestPath.push_back(0);
    reverse(bestPath.begin(), bestPath.end());
    return minCost;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cerr << "Uso: " << argv[0] << " <arquivo.tsp>" << endl;
        return 1;
    }
    readTSPFile(argv[1]);
    int n = cities.size();
    cout << "Cidades: " << n << endl;

    vector<int> path;
    double cost;

    // Força Bruta (n <= 10)
    if (n <= 10) {
        auto start = chrono::high_resolution_clock::now();
        cost = bruteForceTSP(path);
        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> tempo = end - start;
        cout << "Força Bruta: Custo = " << cost << ", Tempo = " << tempo.count() << "s\n";
    } else {
        cout << "Força Bruta não executado (n > 10)\n";
    }

    // Guloso
    auto start = chrono::high_resolution_clock::now();
    cost = greedyTSP(path);
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> tempo = end - start;
    cout << "Guloso: Custo = " << cost << ", Tempo = " << tempo.count() << "s\n";

    // Programação Dinâmica (n <= 20)
    if (n <= 20) {
        start = chrono::high_resolution_clock::now();
        cost = dpTSP(path);
        end = chrono::high_resolution_clock::now();
        tempo = end - start;
        if (cost != -1) {
            cout << "DP: Custo = " << cost << ", Tempo = " << tempo.count() << "s\n";
        } else {
            cout << "DP falhou\n";
        }
    } else {
        cout << "DP não executado (n > 20)\n";
    }

    return 0;
}