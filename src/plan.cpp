#include "plan.h"


Config Plan::get(const int t) const {
    if (configs.empty()) error("Plan is empty; Failed to retrieve agent configurations");
    if (!(0 <= t && t < (int)configs.size())) error("Invalid timestep; Failed to retrieve agent configurations");
    return configs[t];
}

State Plan::get(const int t, const int i) const {
    if (configs.empty()) error("Plan is empty; Failed to retrieve agent state");
    if (!(0 <= t && t < (int)configs.size())) error("Invalid timestep; Failed to retrieve agent state");
    if (!(0 <= i && i < (int)configs.at(0).size())) error("Invalid agent index; Failed to retrieve agent state");
    return configs[t][i];
}

Config Plan::getLast() const {
    // get final state of all agents
    if (configs.empty()) error("Plan is empty; Failed to retrieve final configuration");
    const int N = (int)configs.at(0).size();
    Config config;
    for (int i = 0; i < N; ++i) {
        Path path = getPath(i);
        config.push_back(path.back());
    }
    return config;
}

bool Plan::sameConfig(const Config& a, const Config& b) const {
    if (a.size() != b.size()) return false;
    for (int i = 0; i < (int)a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

bool Plan::validate(const Config& start, const Config& goal) const {
    if (configs.empty()) error("Plan is empty; Nothing to validate");
    // check goal configuration
    if (!sameConfig(getLast(), goal)) {
        warn("Validation failed; Agents did not reach their goal");
        return false;
    }
    // check start configuration
    if (!sameConfig(get(0), start)) {
        warn("Validation failed; Incorrect agent start states");
        return false;
    }
    // check conflicts
    const int N = (int)get(0).size();
    for (int t = 1; t <= getMakespan(); ++t) {
        if ((int)get(t).size() != N) {
            warn("Validation failed; Unknown size of configuration");
            return false;
        }

        for (int i = 0; i < N; ++i) {
            State curr = get(t, i);
            if (curr.node == nullptr) continue;     // agent does not exist on this timestep
            State prev = get(t - 1, i);
            Nodes V = prev.node->neighbor;
            V.push_back(curr.node);
            if (!inArray(curr.node, V)) {
                warn("Validation failed; Agent made an invalid transition");
                return false;
            }
            if (prev.node == curr.node) {
                int dtheta = (curr.orientation - prev.orientation + 4) % 4;
                if (!(dtheta == 0 || dtheta == 1 || dtheta == 3)) {
                    warn("Validation failed; Agent made an invalid rotation");
                    return false;
                }
            } else {
                if (prev.orientation != curr.orientation) {
                    warn("Validation failed; Agent made an invalid move with rotation");
                    return false;
                }
                Pos ds;
                switch (prev.orientation) {
                    case 0 : ds = Pos(0, 1); break;
                    case 1 : ds = Pos(-1, 0); break;
                    case 2 : ds = Pos(0, -1); break;
                    case 3 : ds = Pos(1, 0); break;
                    default : error("Unknown agent orientation");
                }
                if (!(prev.node->pos + ds == curr.node->pos)) {
                    warn("Validation failed; Agent made an invalid move");
                    return false;
                }
            }

            for (int j = i + 1; j < N; ++j) {
                State other_curr = get(t, j);
                State other_prev = get(t - 1, j);
                if (curr.node == other_curr.node) {
                    warn("Validation failed; Vertex conflict");
                    return false;
                }
                if (curr.node == other_prev.node && prev.node == other_curr.node) {
                    warn("Validation failed; Edge conflict");
                    return false;
                }
            }
        }
    }
    return true;
}

void Plan::save(const std::string& filename) const {
    if (configs.empty()) {
        warn("Plan is empty; Nothing to save");
        return;
    }

    std::ofstream myfile;
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for (int i = 0; i < size(); ++i) {
        Path path = getPath(i);
        myfile << "[Agent " << std::right << std::setw(3) << i << "] : ";
        for (auto state : path) {
            myfile << state << " ";
        }
        myfile << std::endl;
    }
    myfile.close();
}

void Plan::add(const Config& c) {
    if (!configs.empty() && configs.at(0).size() != c.size()) {
        error("Failed to add config to plan; Mismatch in size");
    }
    configs.push_back(c);
}

Path Plan::getPath(const int i) const {
    if (configs.empty()) error("Plan is empty; Failed to retrieve agent path");
    if (!(0 <= i && i < (int)configs.at(0).size())) error("Invalid agent index; Failed to retrieve agent path");
    Path path;
    for (int t = 0; t <= getMakespan(); ++t) {
        State state = get(t, i);
        if (state.node == nullptr) break;
        path.push_back(state);
    }
    return path;
}

bool Plan::validate(MAPF_Instance* P) const {
    return validate(P->getConfigStart(), P->getConfigGoal());
}
