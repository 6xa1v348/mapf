#include "problem.h"


void MAPF_Instance::setRandomStartsGoals() {
    // create problem with randomized start and goal states
    config_s.clear(); config_g.clear();
    const int N = G->size();

    std::vector<int> starts(N);
    std::iota(starts.begin(), starts.end(), 0);
    std::shuffle(starts.begin(), starts.end(), *MT);
    int i = 0;
    while (true) {
        while (G->getNode(starts[i]) == nullptr) {
            ++i;
            if (i >= N) error("Too many agents");
        }
        config_s.push_back({G->getNode(starts[i]), getRandomInt(0, 3, *MT)});
        if ((int)config_s.size() == num_agents) break;
        ++i;
    }

    std::vector<int> goals(N);
    std::iota(goals.begin(), goals.end(), 0);
    std::shuffle(goals.begin(), goals.end(), *MT);
    int j = 0;
    while (true) {
        while (G->getNode(goals[j]) == nullptr) {
            ++j;
            if (j >= N) error("Too many agents");
        }
        
        // lazy reinitialization
        if (G->getNode(goals[j]) == config_s[config_g.size()].node) {
            config_g.clear();
            std::shuffle(goals.begin(), goals.end(), *MT);
            j = 0;
            continue;
        }
        config_g.push_back({G->getNode(goals[j]), getRandomInt(0, 3, *MT)});
        if ((int)config_g.size() == num_agents) break;
        ++j;
    }
}

State MAPF_Instance::getStart(int i) const {
    if (!(0 <= i && i < (int)config_s.size())) {
        error("Agent index exceeded number of start states");
    }
    return config_s[i];
}

State MAPF_Instance::getGoal(int i) const {
    if (!(0 <= i && i < (int)config_g.size())) {
        error("Agent index exceeded number of goal states");
    }
    return config_g[i];
}

void MAPF_Instance::make(int num_agents) {
    // make random instance given number of agents
    this->num_agents = num_agents;
    auto t_start = Time::now();
    setRandomStartsGoals();
    info("Generate random starts and goals", t_start);
}

void MAPF_Instance::make(const Config& config_s, const Config& config_g, int num_agents) {
    if (config_s.size() != num_agents) error("Mismatch between number of agents and start states");
    if (config_g.size() != num_agents) error("Mismatch between number of agents and goal states");
    this->num_agents = num_agents;
    this->config_s = config_s;
    this->config_g = config_g;
}
