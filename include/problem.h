#pragma once
#include "logger.h"
#include "graph.h"


using Config = std::vector<State>;
using Configs = std::vector<Config>;

class Problem {
    protected:
        LOGGER(Problem);

        Grid* G;
        std::mt19937* MT;
        int max_timestep;       // maximum number of discrete steps
        int max_comp_time;      // maximum computation time limit (ms)

    public:
        Problem() {}
        Problem(Grid* G, std::mt19937* MT, int max_timestep, int max_comp_time) :
            G(G), MT(MT), max_timestep(max_timestep), max_comp_time(max_comp_time) {}
        virtual ~Problem() = default;

        Grid* getG() const {return G;}
        std::mt19937* getMT() const {return MT;}
        int getMaxTimestep() const {return max_timestep;}
        int getMaxCompTime() const {return max_comp_time;}
        void setMaxTimestep(const int t) {max_timestep = t;}
        void setMaxCompTime(const int t) {max_comp_time = t;}
};

class MAPF_Instance : public Problem {
    private:
        std::string instance_name;
        Config config_s;        // start configuration
        Config config_g;        // goal configuration
        int num_agents;
        
        void setRandomStartsGoals();

    public:
        MAPF_Instance(Grid* G, std::mt19937* MT, int max_timestep, int max_comp_time) :
            Problem(G, MT, max_timestep, max_comp_time), instance_name("custom"), num_agents(0) {}
        ~MAPF_Instance() = default;

        std::string getInstanceFileName() const {return instance_name;}
        int getNum() const {return num_agents;}
        Config getConfigStart() const {return config_s;}
        Config getConfigGoal() const {return config_g;}
        State getStart(int i) const;
        State getGoal(int i) const;

        void make(int num_agents);      // make random instance
        void make(const Config& config_s, const Config& config_g, int num_agents);
};
