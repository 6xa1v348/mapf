#pragma once
#include "logger.h"
#include "problem.h"
#include "plan.h"


class MinimumSolver {
    private:
        int comp_time;
        Time::time_point t_start;

    protected:
        std::string solver_name;
        Grid* const G;
        std::mt19937* const MT;
        const int max_timestep;
        const int max_comp_time;
        Plan solution;
        bool solved;

    public:
        int getRemainedTime() const {
            return std::max(0, max_comp_time - getSolverElapsedTime());
        }
        bool overCompTime() const {
            return getSolverElapsedTime() >= max_comp_time;
        }
        int getSolverElapsedTime() const {
            return getElapsedTime(t_start);
        }

    protected:
        void start() {t_start = Time::now();}
        void end() {comp_time = getSolverElapsedTime();}
        virtual void exec() = 0;

    public:
        MinimumSolver(Problem* P) :
            solver_name(""),
            G(P->getG()),
            MT(P->getMT()),
            max_timestep(P->getMaxTimestep()),
            max_comp_time(P->getMaxCompTime()),
            solved(false),
            comp_time(0) {}
        virtual ~MinimumSolver() {}
        virtual void solve() {
            start(); exec(); end();
        }

        Plan getSolution() const {return solution;}
        bool succeed() const {return solved;}
        std::string getSolverName() const {return solver_name;}
        int getMaxTimestep() const {return max_timestep;}
        int getCompTime() const {return comp_time;}
};

class MAPF_Solver : public MinimumSolver {
    private:
        int LB_soc;     // number of steps
        int LB_makespan;

    protected:
        MAPF_Instance* const P;
        int precomp_time;
        using DistanceTable = std::vector<std::vector<int>>;
        DistanceTable distance_table;       // number of steps to target
    
    private:
        void computeLowerBounds();
        void exec();

    protected:
        virtual void run() {}

    public:
        MAPF_Solver(MAPF_Instance* P) :
            MinimumSolver(P),
            P(P),
            LB_soc(0),
            LB_makespan(0),
            precomp_time(0),
            distance_table(P->getNum(), std::vector<int>(G->size(), max_timestep)) {}
        virtual ~MAPF_Solver() {}

        MAPF_Instance* getP() {return P;}
        int getLowerBoundSOC();
        int getLowerBoundMakespan();
        int getPreCompTime() {return precomp_time;}
        DistanceTable getDistanceTable() {return distance_table;}

        int pathDist(Node* const u, Node* const v) const;       // number of steps from node u to node v
        int pathDist(const int i, Node* const u) const;         // number of steps for agent i from node u
        int pathDist(const int i) const;                        // number of steps for agent i
        void createDistanceTable();
};
