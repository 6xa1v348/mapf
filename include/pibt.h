#pragma once
#include "logger.h"
#include "solver.h"


enum class Action {WAIT, MOVE, TURN_LEFT, TURN_RIGHT, NONE};
using Actions = std::vector<Action>;

class PIBT : public MAPF_Solver {
    private:
        struct Agent {
            int id;
            State curr;
            Node* next;     // next position
            State goal;
            int elapsed;    // number of steps elapsed
            int init_dist;  // distance from start to goal
            float epsilon;
            bool done;
        };
        using Agents = std::vector<Agent*>;

        Agents occupied_now;    // current locations
        Agents occupied_next;   // next locations
        bool distance_initialized;

        bool funcPIBT(Agent* a, Agent* b = nullptr);
        Action getAction(const State& curr, Node* const next, const State& goal) const;
        void run();

        void wait(Agent* a, Config& config);
        void turn(Agent* a, const Actions& actions, Config& config);
        bool move(Agent* a, const Actions& actions, Config& config);

    protected:
        LOGGER(PIBT);

    public:
        PIBT(MAPF_Instance* P) :
            MAPF_Solver(P),
            occupied_now(Agents(G->size(), nullptr)),
            occupied_next(Agents(G->size(), nullptr)),
            distance_initialized(false) {
                solver_name = "PIBT";
            }
        ~PIBT() {}
};
