#include "pibt.h"


bool PIBT::funcPIBT(Agent* a, Agent* b) {
    auto compare = [&](Node* const u, Node* const v) {
        int du = pathDist(a->id, u);        // distance-to-goal
        int dv = pathDist(a->id, v);        // distance-to-goal
        if (du != dv) return du < dv;

        // prefer forward movement
        Pos pos;
        switch (a->curr.orientation) {
            case 0 : pos = a->curr.node->pos + Pos(0, 1); break;
            case 1 : pos = a->curr.node->pos - Pos(1, 0); break;
            case 2 : pos = a->curr.node->pos - Pos(0, 1); break;
            case 3 : pos = a->curr.node->pos + Pos(1, 0); break;
            default : error("Unknown agent orientation");
        }
        if (u->pos == pos && v->pos != pos) {
            return true;
        }
        if (u->pos != pos && v->pos == pos) {
            return false;
        }

        // prefer empty nodes
        if (occupied_now[u->id] != nullptr && occupied_now[v->id] == nullptr) {
            return false;
        }
        if (occupied_now[u->id] == nullptr && occupied_now[v->id] != nullptr) {
            return true;
        }
        return false;
    };

    Nodes V;
    for (auto n : a->curr.node->neighbor) {
        if (G->getWeight(a->curr.node, n) < MAX_WEIGHT) {
            V.push_back(n);
        }
    }
    V.push_back(a->curr.node);
    std::shuffle(V.begin(), V.end(), *MT);
    std::sort(V.begin(), V.end(), compare);     // ranking preference

    for (auto v : V) {
        if (occupied_next[v->id] != nullptr) continue;      // target node not available
        if (b != nullptr && v == b->curr.node) continue;    // swap conflict
        occupied_next[v->id] = a;
        a->next = v;
        auto k = occupied_now[v->id];
        if (k != nullptr && k->next == nullptr) {
            if (!funcPIBT(k, a)) continue;
        }
        return true;
    }
    
    // no viable move, wait
    a->next = a->curr.node;
    occupied_next[a->next->id] = a;
    return false;
}

Action PIBT::getAction(const State& curr, Node* const next, const State& goal) const {
    if (curr.node == nullptr || next == nullptr || goal.node == nullptr) {
        error("Failed to retrieve agent action");
    }
    if (next == curr.node) {
        if (next == goal.node){
            // agent is at goal, rotate to face target orientation
            int dtheta = (goal.orientation - curr.orientation + 4) % 4;
            if (dtheta == 1 || dtheta  == 2) {
                return Action::TURN_LEFT;
            } else {
                return Action::TURN_RIGHT;
            }
        }
        return Action::WAIT;
    }
    if (curr.orientation == -1) return Action::MOVE;

    int target;
    if (next->pos == curr.node->pos + Pos(0, 1)) {
        target = 0;
    } else if (next->pos == curr.node->pos - Pos(1, 0)) {
        target = 1;
    } else if (next->pos == curr.node->pos - Pos(0, 1)) {
        target = 2;
    } else if (next->pos == curr.node->pos + Pos(1, 0)) {
        target = 3;
    } else {
        error("Agent intent to make an invalid move");
    }
    if (curr.orientation == target) {
        return Action::MOVE;
    } else {
        int dtheta = (target - curr.orientation + 4) % 4;
        if (dtheta == 1 || dtheta == 2) {
            return Action::TURN_LEFT;
        } else {
            return Action::TURN_RIGHT;
        }
    }
}

void PIBT::wait(Agent* a, Config& config) {
    if (occupied_next[a->next->id] != a) error("Inconsistent plan");
    occupied_next[a->next->id] = nullptr;
    a->next = nullptr;
    config[a->id] = a->curr;
}

void PIBT::turn(Agent* a, const Actions& actions, Config& config) {
    if (occupied_next[a->next->id] != a) error("Inconsistent plan");
    occupied_next[a->next->id] = nullptr;
    a->next = nullptr;
    int h;
    if (actions[a->id] == Action::TURN_LEFT) {
        h = (a->curr.orientation + 1) % 4;
    } else if (actions[a->id] == Action::TURN_RIGHT) {
        h = (a->curr.orientation + 3) % 4;
    } else {
        error("Incorrect action resolution");
    }
    a->curr = State(a->curr.node, h);
    config[a->id] = a->curr;
}

bool PIBT::move(Agent* a, const Actions& actions, Config& config) {
    if (occupied_next[a->next->id] != a) error("Inconsistent plan");
    
    if (occupied_now[a->next->id] == nullptr) {
        // target node is available, move
        if (occupied_now[a->curr.node->id] != a) error("Inconsistent plan");
        occupied_now[a->curr.node->id] = nullptr;
        occupied_now[a->next->id] = a;
        int h = a->curr.orientation;
        a->curr = State(a->next, h);
        occupied_next[a->next->id] = nullptr;
        a->next = nullptr;
        config[a->id] = a->curr;
        return true;
    } else {
        // target node is currently occupied, verify
        auto b = occupied_now[a->next->id];
        if (actions[b->id] != Action::MOVE || b->next == nullptr) {
            // other agent does not intent to move, or already moved
            wait(a, config);
            return false;
        } else if (actions[b->id] == Action::MOVE && b->next != nullptr) {
            // move other agent
            if (occupied_now[a->curr.node->id] != a) error("Inconsistent plan");
            occupied_now[a->curr.node->id] = nullptr;       // temporarily release current node
            if (!move(b, actions, config)) {
                // other agent failed to move, wait
                occupied_now[a->curr.node->id] = a;
                wait(a, config);
                return false;
            }
            // other agent moved, move
            if (occupied_now[a->next->id] != nullptr) error("Inconsistent plan");
            occupied_now[a->next->id] = a;
            int h = a->curr.orientation;
            a->curr = State(a->next, h);
            occupied_next[a->next->id] = nullptr;
            a->next = nullptr;
            config[a->id] = a->curr;
            return true;
        }
    }
    return false;
}

void PIBT::run() {
    info("Running PIBT...");
    auto compare = [](Agent* a, Agent* b) {
        if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;           // priority based on elapsed time
        if (a->init_dist != b->init_dist) return a->init_dist > b->init_dist;   // priority based on initial distance-to-goal
        return a->epsilon > b->epsilon;
    };

    Agents A;
    std::fill(occupied_now.begin(), occupied_now.end(), nullptr);
    std::fill(occupied_next.begin(), occupied_next.end(), nullptr);
    // initialize
    for (int i = 0;i < P->getNum(); ++i) {
        State s = P->getStart(i);
        State g = P->getGoal(i);
        int init_dist = distance_initialized ? pathDist(i) : 0;
        Agent* a = new Agent{i, s, nullptr, g, 0, init_dist, getRandomFloat(0, 1, *MT), false};
        A.push_back(a);
        occupied_now[a->curr.node->id] = a;
    }
    solution.add(P->getConfigStart());
    int timestep = 0;

    std::sort(A.begin(), A.end(), compare);     // sort agents by priority
    while (true) {
        for (auto a : A) {
            if (a->done) continue;
            if (a->next == nullptr) {
                funcPIBT(a);
            }
        }

        // convert PIBT solution to actions
        Actions actions(P->getNum(), Action::NONE);
        for (auto a : A) {
            if (a->done) continue;
            actions[a->id] = getAction(a->curr, a->next, a->goal);
        }

        // update configs
        Config config(P->getNum());
        for (auto a : A) {
            if (a->done) continue;
            if (a->next == nullptr) continue;       // already updated
            if (actions[a->id] == Action::WAIT) {
                wait(a, config);
                a->elapsed += 1;
            } else if (actions[a->id] == Action::TURN_LEFT || actions[a->id] == Action::TURN_RIGHT) {
                turn(a, actions, config);
                a->elapsed += 1;
            } else if (actions[a->id] == Action::MOVE) {
                move(a, actions, config);
                a->elapsed += 1;
            } else {
                error("Unknown agent action");
            }
        }
        solution.add(config);

        bool done = true;
        for (auto a : A) {
            if (a->done) continue;
            if (a->curr == a->goal) {
                // remove agent
                if (occupied_now[a->curr.node->id] != a) error("Inconsistent plan");
                occupied_now[a->curr.node->id] = nullptr;
                a->done = true;
            }
            done &= a->done;
        }
        ++timestep;
        if (done) {
            solved = true;
            break;
        }

        if (timestep >= max_timestep) {
            warn("Exceeded maximum number of timesteps");
            break;
        } else if (overCompTime()) {
            warn("Exceeded maximum computation time limit");
            break;
        }
    }

    // release memory
    for (auto a : A) {
        delete a;
    }
}
