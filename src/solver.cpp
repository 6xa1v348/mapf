#include "solver.h"


void MAPF_Solver::computeLowerBounds() {
    LB_soc = 0;
    LB_makespan = 0;
    for (int i = 0; i < P->getNum(); ++i) {
        int d = pathDist(i);
        LB_soc += d;
        if (d > LB_makespan) LB_makespan = d;
    }
}

void MAPF_Solver::exec() {
    createDistanceTable();
    precomp_time = getSolverElapsedTime();
    run();
}

int MAPF_Solver::getLowerBoundSOC() {
    if (LB_soc == 0) computeLowerBounds();
    return LB_soc;
}

int MAPF_Solver::getLowerBoundMakespan() {
    if (LB_makespan == 0) computeLowerBounds();
    return LB_makespan;
}

int MAPF_Solver::pathDist(Node* const u, Node* const v) const {
    if (u == v) return 0;
    auto [path, cost] = G->getPathWithCost(State(u), State(v), MT);
    return path.size() - 1;
}

int MAPF_Solver::pathDist(const int i, Node* const u) const {
    return distance_table[i][u->id];
}

int MAPF_Solver::pathDist(const int i) const {
    return pathDist(i, P->getStart(i).node);
}

void MAPF_Solver::createDistanceTable() {
    // for each agent, precompute distance-to-goal using backward dijkstra
    using cmp = std::tuple<float, int, Node*>;      // <cost, step, node>
    std::vector<float> tmp(G->size());      // temporary cost-map
    distance_table.resize(P->getNum(), std::vector<int>(G->size(), max_timestep));
    for (int i = 0; i < P->getNum(); ++i) {
        // initialize cost-map
        tmp.assign(tmp.size(), MAX_WEIGHT);
        std::priority_queue<cmp, std::vector<cmp>, std::greater<>> OPEN;
        Node* g = P->getGoal(i).node;
        distance_table[i][g->id] = 0;
        tmp[g->id] = 0.f;
        OPEN.push({0.f, 0, g});
        while (!OPEN.empty()) {
            auto [cn, dn, n] = OPEN.top(); OPEN.pop();
            if (cn > tmp[n->id]) continue;
            for (auto m : n->neighbor) {
                if (G->getWeight(m, n) >= MAX_WEIGHT) continue;
                float cm = cn + G->getWeight(m, n);
                int dm = dn + 1;
                if (cm < tmp[m->id]) {
                    tmp[m->id] = cm;
                    distance_table[i][m->id] = dm;
                    OPEN.push({cm, dm, m});
                }

            }
        }
    }
}