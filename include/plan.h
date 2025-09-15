#pragma once
#include "logger.h"
#include "problem.h"


struct Plan {
    private:
        Configs configs;

        Config get(const int t) const;
        State get(const int t, const int i) const;
        Config getLast() const;

        bool sameConfig(const Config& a, const Config& b) const;
        bool validate(const Config& start, const Config& goal) const;

    protected:
        LOGGER(Plan);

    public:
        ~Plan() {}

        bool empty() const {return configs.empty();}
        int size() const {
            if (configs.empty()) return 0;
            return (int)configs.at(0).size();
        }
        int getMakespan() const {
            if (configs.empty()) return 0;
            return (int)configs.size() - 1;
        }
        void save(const std::string& filename = "output.plan") const;

        void add(const Config& c);
        Path getPath(const int i) const;
        bool validate(MAPF_Instance* P) const;
};

using Plans = std::vector<Plan>;
