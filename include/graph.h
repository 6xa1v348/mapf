#pragma once
#include "common.h"
#include "logger.h"


struct Pos {
    int x;
    int y;

    Pos() {}
    Pos(int x, int y) :
        x(x), y(y) {}
    ~Pos() {}

    int manhattan(const Pos& pos) const;
    float euclidean(const Pos& pos) const;

    bool operator==(const Pos& other) const {
        return other.x == x &&
            other.y == y;
    }

    bool operator!=(const Pos& other) const {
        return !(*this == other);
    }

    Pos operator+(const Pos& other) const {
        return Pos(x + other.x, y + other.y);
    }

    Pos operator-(const Pos& other) const {
        return Pos(x - other.x, y - other.y);
    }

    void operator+=(const Pos& other) {
        x = x + other.x;
        y = y + other.y;
    }

    void operator-(const Pos& other) {
        x = x - other.x;
        y = y - other.y;
    }
};

struct Node;
using Nodes = std::vector<Node*>;

struct Node {
    const int id;
    const Pos pos;
    Nodes neighbor;

    Node(int id, int x, int y) :
        id(id), pos(Pos(x, y)), neighbor(Nodes(0)) {}
    ~Node() {}

    int getDegree() const;
    int manhattan(const Node& node) const;
    int manhattan(Node* const node) const;
    float euclidean(const Node& node) const;
    float euclidean(Node* const node) const;

    bool operator==(const Node& other) const {
        return other.id == id;
    }

    bool operator!=(const Node& other) const {
        return other.id != id;
    }
    
    bool operator==(Node* const other) const {
        return other->id == id;
    }

    bool operator!=(Node* const other) const {
        return other->id != id;
    }
};

struct State {
    Node* node;
    int orientation;

    State() : node(nullptr), orientation(-1) {}
    State(Node* node, int orientation = -1) :
        node(node), orientation(orientation) {}

    struct Hasher {
        size_t operator()(const State& s) const {
            int id = (s.node != nullptr) ? s.node->id : -1;
            size_t h1 = std::hash<int>()(id);
            size_t h2 = std::hash<int>()(s.orientation);
            return h1 ^ (h2 << 1);
        }
    };

    bool operator==(const State& other) const {
        return node == other.node &&
            orientation == other.orientation;
    }

    bool operator!=(const State& other) const {
        return !(*this == other);
    }
};

using Path = std::vector<State>;
std::ostream& operator<<(std::ostream& os, const Pos& pos);
std::ostream& operator<<(std::ostream& os, const Node& node);
std::ostream& operator<<(std::ostream& os, const State& state);

class Grid {
    private:
        std::string map_file;
        int height;
        int width;
        int channels;       // number of moves

        Nodes V;
        std::vector<float> weights;

    protected:
        LOGGER(Grid);

    public:
        Grid(const std::string& map_file, bool load_weights = false);
        ~Grid() {}

        std::string getMapFileName() const {return map_file;}
        int getHeight() const {return height;}
        int getWidth() const {return width;}
        int getChannels() const {return channels;}
        const std::vector<float>& getWeights() const {return weights;}
        void setWeights(const std::vector<float>& weights);
        int size() const {return height * width;}

        float getWeight(int x, int y, int ch) const;
        float getWeight(Node* const u, int ch) const;
        float getWeight(Node* const u, Node* const v) const;
        int getNeighbor(const State& s, std::array<State, 4>& buf) const;
        float dist(Node* const u, Node* const v) const {return (float)u->manhattan(v);}

        bool existNode(int id) const;
        bool existNode(int x, int y) const;
        Node* getNode(int id) const {return V[id];}
        Node* getNode(int x, int y) const {return getNode(y * width + x);}
        std::pair<Path, float> getPathWithCost(const State& s, const State& g, std::mt19937* MT = nullptr, const Nodes& prohibited = {}) const;
};
