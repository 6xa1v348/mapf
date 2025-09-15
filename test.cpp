#include <cassert>
#include <filesystem>

#include "logger.h"
#include "graph.h"
#include "problem.h"
#include "solver.h"
#include "pibt.h"


template <typename... Args>
inline void debug(const std::string& msg, Args&&... args) {
    Logger::get().log(LogLevel::DEBUG, "TEST", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const std::string& msg, Args&&... args) {
    Logger::get().log(LogLevel::WARN, "TEST", msg, std::forward<Args>(args)...);
}

void test_graph() {
    auto t_start = Time::now();

    Grid* G = new Grid("assets/warehouse", false);
    assert(G->getMapFileName() == "assets/warehouse");
    assert(G->getHeight() == 21);
    assert(G->getWidth() == 35);
    assert(G->getChannels() == 0);
    assert(G->size() == 735);
    assert(G->getWeights().size() == 0);

    std::vector<float> weights(2940, 1.f);
    G->setWeights(weights);
    assert(G->getChannels() == 4);
    assert(G->getWeights().size() == 2940);

    debug("Graph without weights ... [OK]", t_start);
    delete G;

    t_start = Time::now();
    G = new Grid("assets/warehouse", true);
    assert(G->existNode(0, 0) == true);
    assert(G->existNode(7, 2) == false);
    assert(G->getNode(0, 0)->pos == Pos(0, 0));
    assert(G->getNode(7, 2) == nullptr);
    
    assert(G->getWeights().size() == 2940);
    assert(G->getWeight(0, 0, 0) == 1);
    assert(G->getWeight(0, 0, 1) == MAX_WEIGHT);
    assert(G->getWeight(7, 2, 0) == MAX_WEIGHT);
    assert(G->getWeight(6, 2, 1) == 1);
    assert(G->getWeight(6, 2, 3) == MAX_WEIGHT);
    assert(G->getWeight(G->getNode(0, 0), 0) == 1);
    assert(G->getWeight(G->getNode(0, 0), 1) == MAX_WEIGHT);
    assert(G->getWeight(G->getNode(6, 2), 1) == 1);
    assert(G->getWeight(G->getNode(6, 2), 3) == MAX_WEIGHT);
    assert(G->getWeight(G->getNode(6, 2), G->getNode(6, 3)) == 1);

    std::array<State, 4> buf;
    assert(G->getNeighbor(State(G->getNode(6, 2), 1), buf) == 3);
    assert(G->getNeighbor(State(G->getNode(6, 2), 3), buf) == 2);
    assert(G->getNeighbor(State(G->getNode(6, 2)), buf) == 3);
    assert(G->getNeighbor(State(G->getNode(0, 0), 0), buf) == 3);
    assert(G->getNeighbor(State(G->getNode(0, 0)), buf) == 2);
    assert(G->getNeighbor(State(G->getNode(1, 1), 1), buf) == 3);
    assert(G->getNeighbor(State(G->getNode(1, 1)), buf) == 4);

    assert(G->dist(G->getNode(0, 0), G->getNode(34, 20)) == 54);
    assert(G->getPathWithCost(State(G->getNode(0, 0), 0), State(G->getNode(34, 20), 3)).first.size() == 56);
    assert(G->getPathWithCost(State(G->getNode(0, 0), 0), State(G->getNode(34, 20), 3)).second == 55);
    assert(G->getPathWithCost(State(G->getNode(0, 0), 0), State(G->getNode(34, 20), 2)).first.size() == 57);
    assert(G->getPathWithCost(State(G->getNode(0, 0), 0), State(G->getNode(34, 20), 2)).second == 56);
    assert(G->getPathWithCost(State(G->getNode(0, 0)), State(G->getNode(34, 20))).first.size() == 55);
    assert(G->getPathWithCost(State(G->getNode(0, 0)), State(G->getNode(34, 20))).second == 54);
    debug("Graph with weights ... [OK]", t_start);
    delete G;    
}

void test_problem() {
    auto t_start = Time::now();

    Grid* G = new Grid("assets/warehouse", true);
    std::mt19937* MT = new std::mt19937(42);
    int max_timestep = 10000;
    int max_comp_time = 1000;
    MAPF_Instance* P = new MAPF_Instance(G, MT, max_timestep, max_comp_time);
    assert(P->getG() == G);
    assert(P->getMT() == MT);
    assert(P->getInstanceFileName() == "custom");
    assert(P->getMaxTimestep() == 10000);
    assert(P->getMaxCompTime() == 1000);
    assert(P->getNum() == 0);
    assert(P->getConfigStart().size() == 0);
    assert(P->getConfigGoal().size() == 0);
    
    P->setMaxTimestep(50000);
    P->setMaxCompTime(2000);
    assert(P->getMaxTimestep() == 50000);
    assert(P->getMaxCompTime() == 2000);

    P->make(200);
    assert(P->getNum() == 200);
    assert(P->getConfigStart().size() == 200);
    assert(P->getConfigGoal().size() == 200);
    debug("Random problem instance ... [OK]", t_start);

    t_start = Time::now();
    Config config_s{
        {G->getNode(0, 0), 0},
        {G->getNode(27, 1), 2},
        {G->getNode(0, 10), 3},
    };
    Config config_g{
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
        {G->getNode(34, 10), 1},
    };
    P->make(config_s, config_g, 3);
    assert(P->getNum() == 3);
    assert(P->getConfigStart().size() == 3);
    assert(P->getConfigGoal().size() == 3);
    assert(P->getStart(0) == State(G->getNode(0, 0), 0));
    assert(P->getGoal(0) == State(G->getNode(17, 18), 0));
    assert(P->getStart(1) == State(G->getNode(27, 1), 2));
    assert(P->getGoal(1) == State(G->getNode(17, 18), 0));
    assert(G->getPathWithCost(P->getStart(0), P->getGoal(0)).first.size() == 38);
    assert(G->getPathWithCost(P->getStart(0), P->getGoal(0)).second == 37);
    debug("Custom problem instance ... [OK]", t_start);
    delete P; delete G; delete MT;
}

void test_solver() {
    auto t_start = Time::now();

    Grid* G = new Grid("assets/warehouse", true);
    std::mt19937* MT = new std::mt19937(42);
    int max_timestep = 10000;
    int max_comp_time = 1000;
    MAPF_Instance* P = new MAPF_Instance(G, MT, max_timestep, max_comp_time);
    P->make(100);
    MAPF_Solver* baseline = new MAPF_Solver(P);

    assert(baseline->succeed() == false);
    assert(baseline->getSolverName() == "");
    assert(baseline->getMaxTimestep() == 10000);
    assert(baseline->getPreCompTime() == 0);
    assert(baseline->getCompTime() == 0);
    assert(baseline->getP() == P);
    assert(baseline->getDistanceTable().size() == 100);

    auto D1 = baseline->getDistanceTable();
    int sum = 0;
    for (int i = 0; i < P->getNum(); ++i) {
        sum += std::accumulate(D1[i].begin(), D1[i].end(), 0);
    }
    assert(sum == 735000000);

    baseline->createDistanceTable();
    auto D2 = baseline->getDistanceTable();
    sum = 0;
    for (int i = 0; i < P->getNum(); ++i) {
        sum += std::accumulate(D2[i].begin(), D2[i].end(), 0);
    }
    assert(sum < 735000000);

    baseline->solve();
    assert(baseline->getPreCompTime() != 0);
    assert(baseline->getCompTime() != 0);
    assert(baseline->getDistanceTable().size() == 100);
    assert(baseline->getLowerBoundSOC() != 0);
    assert(baseline->getLowerBoundMakespan() != 0);
    assert(baseline->succeed() == false);
    debug("Baseline solver ... [OK]", t_start);
    delete baseline; delete P; delete G; delete MT;
}

void test_pibt() {
    auto t_start = Time::now();

    Grid* G = new Grid("assets/warehouse", true);
    std::mt19937* MT = new std::mt19937(42);
    int max_timestep = 10000;
    int max_comp_time = 1000;
    MAPF_Instance* P = new MAPF_Instance(G, MT, max_timestep, max_comp_time);
    P->make(200);
    MAPF_Solver* mapf = new PIBT(P);

    assert(mapf->getSolverName() == "PIBT");
    assert(mapf->getMaxTimestep() == 10000);
    assert(mapf->getPreCompTime() == 0);
    assert(mapf->getCompTime() == 0);
    assert(mapf->getP() == P);
    assert(mapf->getDistanceTable().size() == 200);
    assert(mapf->getSolution().size() == 0);
    assert(mapf->succeed() == false);

    mapf->solve();
    assert(mapf->getPreCompTime() != 0);
    assert(mapf->getCompTime() != 0);
    assert(mapf->getSolution().size() == 200);
    assert(mapf->succeed() == true);
    assert(mapf->getSolution().validate(P) == true);
    
    assert(mapf->getSolution().getMakespan() != 0);
    assert(mapf->getLowerBoundMakespan() <= mapf->getSolution().getMakespan());
    assert(mapf->getLowerBoundSOC() != 0);
    debug("PIBT solver (random instance) ... [OK]", t_start);
    delete mapf;

    // scenario 1
    t_start = Time::now();
    Config config_s{
        {G->getNode(9, 17), 3},
        {G->getNode(25, 17), 1},
    };
    Config config_g{
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
    };
    P->make(config_s, config_g, 2);
    mapf = new PIBT(P);
    assert(mapf->getPreCompTime() == 0);
    assert(mapf->getCompTime() == 0);
    assert(mapf->getP() == P);
    assert(mapf->getDistanceTable().size() == 2);
    assert(mapf->getSolution().size() == 0);
    assert(mapf->succeed() == false);

    mapf->solve();
    assert(mapf->getPreCompTime() != 0);
    assert(mapf->getCompTime() != 0);
    assert(mapf->getSolution().size() == 2);
    assert(mapf->succeed() == true);
    assert(mapf->getSolution().validate(P) == true);
    mapf->getSolution().save("scenario1.plan");
    debug("PIBT solver (Scenario 1) ... [OK]", t_start);
    delete mapf;

    // scenario 2
    t_start = Time::now();
    config_s = {
        {G->getNode(9, 17), 3},
        {G->getNode(25, 17), 1},
        {G->getNode(17, 9), 0},
    };
    config_g = {
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
    };
    P->make(config_s, config_g, 3);
    mapf = new PIBT(P);
    assert(mapf->getPreCompTime() == 0);
    assert(mapf->getCompTime() == 0);
    assert(mapf->getP() == P);
    assert(mapf->getDistanceTable().size() == 3);
    assert(mapf->getSolution().size() == 0);
    assert(mapf->succeed() == false);

    mapf->solve();
    assert(mapf->getPreCompTime() != 0);
    assert(mapf->getCompTime() != 0);
    assert(mapf->getSolution().size() == 3);
    assert(mapf->succeed() == true);
    assert(mapf->getSolution().validate(P) == true);
    mapf->getSolution().save("scenario2.plan");
    debug("PIBT solver (Scenario 2) ... [OK]", t_start);
    delete mapf;

    // scenario 3
    t_start = Time::now();
    config_s = {
        {G->getNode(9, 17), 3},
        {G->getNode(25, 17), 1},
        {G->getNode(17, 9), 0},
        {G->getNode(13, 19), 3},
    };
    config_g = {
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
        {G->getNode(17, 18), 0},
        {G->getNode(17, 15), 2},
    };
    P->make(config_s, config_g, 4);
    mapf = new PIBT(P);
    assert(mapf->getPreCompTime() == 0);
    assert(mapf->getCompTime() == 0);
    assert(mapf->getP() == P);
    assert(mapf->getDistanceTable().size() == 4);
    assert(mapf->getSolution().size() == 0);
    assert(mapf->succeed() == false);

    mapf->solve();
    assert(mapf->getPreCompTime() != 0);
    assert(mapf->getCompTime() != 0);
    assert(mapf->getSolution().size() == 4);
    assert(mapf->succeed() == true);
    assert(mapf->getSolution().validate(P) == true);
    mapf->getSolution().save("scenario3.plan");
    debug("PIBT solver (Scenario 3) ... [OK]", t_start);
    delete mapf; delete P; delete G; delete MT;
}

int main() {
    const std::string testFileName = "test.log";
    if (std::filesystem::exists(testFileName)) {
        std::filesystem::remove(testFileName);
    }
    Logger::get().setVerbose(true);
    Logger::get().enableFileLogging(testFileName);
    auto t = Time::now();

    debug("Starting test ... ");
    test_graph();
    test_problem();
    test_solver();
    test_pibt();
    debug("Test complete", t);
    return 0;
}