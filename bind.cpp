#include <cassert>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>

#include "logger.h"
#include "graph.h"
#include "problem.h"
#include "solver.h"
#include "pibt.h"


struct Parameters {
    bool verbose = true;
    bool log = false;
    std::string map = "";
    bool with_weights = true;
    std::string solver = "";
    int seed = 42;
    int max_timestep = 10000;       // maximum number of discrete steps
    int max_comp_time = 1000;       // maximum computation time limit (ms)
};

void setLogger(bool enabled, bool log){
    Logger::get().setVerbose(enabled);
    if (log) {
        Logger::get().enableFileLogging("output.log");
    } else {
        Logger::get().disableFileLogging();
    }
}

Grid* make_graph(const Parameters& params) {
    setLogger(params.verbose, params.log);
    Grid *G = new Grid(params.map, params.with_weights);
    return G;
}

MAPF_Instance* make_instance(Grid* G, const Parameters& params) {
    std::mt19937* MT = new std::mt19937(42);
    MAPF_Instance* P = new MAPF_Instance(G, MT, params.max_timestep, params.max_comp_time);
    return P;
}

MAPF_Solver* make_solver(MAPF_Instance* P, const Parameters& params) {
    if (params.solver == "PIBT") {
        MAPF_Solver* solver = new PIBT(P);
        return solver;
    }
    throw std::runtime_error("Unknown solver selected");
}

namespace py = pybind11;

PYBIND11_MODULE(mapf, m) {
    py::class_<Parameters>(m, "Parameters", py::dynamic_attr())
        .def(py::init<>())
        .def_readwrite("verbose", &Parameters::verbose)
        .def_readwrite("log", &Parameters::log)
        .def_readwrite("map", &Parameters::map)
        .def_readwrite("with_weights", &Parameters::with_weights)
        .def_readwrite("solver", &Parameters::solver)
        .def_readwrite("seed", &Parameters::seed)
        .def_readwrite("max_timestep", &Parameters::max_timestep)
        .def_readwrite("max_comp_time", &Parameters::max_comp_time);

    py::class_<Grid>(m, "Graph")
        .def("weights", [](const Grid& self) {
            const auto& vec = self.getWeights();
            return py::array_t<float>(vec.size(), vec.data());
        })
        .def("set_weights", [](Grid& self, py::array_t<float> arr) {
            py::buffer_info info = arr.request();
            if (info.ndim != 1) throw std::runtime_error("Weights must be a 1D np array");
            float* data_ptr = static_cast<float*>(info.ptr);
            std::vector<float> vec(data_ptr, data_ptr + info.size);
            self.setWeights(vec);
        }, py::arg("arr"));

    py::class_<Plan>(m, "Plan")
        .def(py::init<>())
        .def("empty", &Plan::empty)
        .def("size", &Plan::size)
        .def("get_makespan", &Plan::getMakespan)
        .def("save", [](Plan& self, const std::string& filename = "output.plan") {
            self.save(filename);
        }, py::arg("filename"))
        .def("validate", [](Plan& self, MAPF_Instance* P) {
            return self.validate(P);
        }, py::arg("instance"))
        .def("get_path", [](Plan& self, int i) {
            Path path = self.getPath(i);
            std::vector<std::tuple<int, int, int>> arr;
            for (auto& s : path) {
                arr.emplace_back(s.node->pos.x, s.node->pos.y, s.orientation);
            }
            return arr;
        }, py::arg("i"));

    py::class_<MAPF_Instance>(m, "Instance")
        .def_property_readonly("name", [](const MAPF_Instance& self) {
            return self.getInstanceFileName();
        })
        .def_property_readonly("num_agents", [](const MAPF_Instance& self) {
            return self.getNum();
        })
        .def("get_starts", [](const MAPF_Instance& self) {
            Config config_s = self.getConfigStart();
            std::vector<std::tuple<int, int, int>> start;
            for (auto& s : config_s) {
                start.emplace_back(s.node->pos.x, s.node->pos.y, s.orientation);
            }
            return start;
        })
        .def("get_goals", [](const MAPF_Instance& self) {
            Config config_g = self.getConfigGoal();
            std::vector<std::tuple<int, int, int>> goal;
            for (auto& s : config_g) {
                goal.emplace_back(s.node->pos.x, s.node->pos.y, s.orientation);
            }
            return goal;
        })
        .def("make", py::overload_cast<int> (&MAPF_Instance::make))
        .def("make", [](MAPF_Instance& self,
            const std::vector<std::tuple<int, int, int>>& start,
            const std::vector<std::tuple<int, int, int>>& goal,
            int num_agents) {
                assert(start.size() == num_agents);
                assert(goal.size() == num_agents);
                Config config_s;
                int x, y, orientation;
                for (auto& s : start) {
                    std::tie(x, y, orientation) = s;
                    assert(0 <= orientation && orientation < 4);
                    config_s.emplace_back(self.getG()->getNode(x, y), orientation);
                }
                Config config_g;
                for (auto& s : goal) {
                    std::tie(x, y, orientation) = s;
                    assert(0 <= orientation && orientation < 4);
                    config_g.emplace_back(self.getG()->getNode(x, y), orientation);
                }
                return self.make(config_s, config_g, num_agents);
            }, py::arg("start"), py::arg("goal"), py::arg("num_agents"));
    
    py::class_<MAPF_Solver>(m, "MAPF_Solver")
        .def_property_readonly("name", [](const MAPF_Solver& self) {
            return self.getSolverName();
        })
        .def("solve", &MAPF_Solver::solve)
        .def("succeed", &MAPF_Solver::succeed)
        .def("get_solution", &MAPF_Solver::getSolution);

    m.def("make_graph", [](const Parameters& params) {
        return std::unique_ptr<Grid>(make_graph(params));
    }, py::arg("params"));

    m.def("make_instance", [](Grid* G, const Parameters& params) {
        return std::unique_ptr<MAPF_Instance>(make_instance(G, params));
    }, py::arg("graph"), py::arg("params"));

    m.def("make_solver", [](MAPF_Instance* P, const Parameters& params) {
        return std::unique_ptr<MAPF_Solver>(make_solver(P, params));
    }, py::arg("instance"), py::arg("params"));
}