#include "graph.h"


int Pos::manhattan(const Pos& pos) const {
    return std::abs(x - pos.x) + std::abs(y - pos.y);
}

float Pos::euclidean(const Pos& pos) const {
    float dx = x - pos.x;
    float dy = y - pos.y;
    return std::sqrt(dx * dx + dy * dy);
}

int Node::getDegree() const {
    return neighbor.size();
}

int Node::manhattan(const Node& node) const {
    return pos.manhattan(node.pos);
}

int Node::manhattan(Node* const node) const {
    return pos.manhattan(node->pos);
}

float Node::euclidean(const Node& node) const {
    return pos.euclidean(node.pos);
}

float Node::euclidean(Node* const node) const {
    return pos.euclidean(node->pos);
}

std::ostream& operator<<(std::ostream& os, const Pos& pos) {
    os << "(" << std::right << std::setw(3) << pos.x << ","
        << std::right << std::setw(3) << pos.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << "node=[" << std::right << std::setw(6) << node.id << "]=<pos: "
        << node.pos << ", neighbor: d=" << std::setw(1) << node.getDegree() << ", n=";
    for (auto v : node.neighbor) {
        os << v->pos << ",";
    }
    os << ">";
    return os;
}

std::ostream& operator<<(std::ostream& os, const State& state) {
    os << "(" << std::right << std::setw(3) << state.node->pos.x << ","
        << std::right << std::setw(3) << state.node->pos.y << ","
        << std::right << std::setw(3) << state.orientation << ")";
    return os;
}

Grid::Grid(const std::string& map_file, bool load_weights) : map_file(map_file) {
    // load graph using map file
    auto t_start = Time::now();

    std::ifstream file(map_file + ".map");
    if (!file) error("File " + map_file + ".map is not found");
    
    // regex for extracting map information
    std::string line;
    std::smatch match;
    std::regex r_height(R"(height\s(\d+))");
    std::regex r_width(R"(width\s(\d+))");
    std::regex r_channels(R"(channels\s(\d+))");
    std::regex r_map(R"(map)");

    // read specifications
    while (getline(file, line)) {
        if (*(line.end() - 1) == 0x0d) line.pop_back();
        if (std::regex_match(line, match, r_height)) {
            height = std::stoi(match[1].str());
            continue;
        }
        if (std::regex_match(line, match, r_width)) {
            width = std::stoi(match[1].str());
            continue;
        }
        if (std::regex_match(line, match, r_map)) {
            break;
        }
    }
    if (!(height > 0 && width > 0)) error("Failed to load map; Nonzero height/width");

    // generate nodes
    int y = 0;
    V = Nodes(height * width, nullptr);
    while (getline(file, line)) {
        if (*(line.end() - 1) == 0x0d) line.pop_back();
        if ((int)line.size() != width) error ("Mismatch in width");
        for (int x = 0; x < width; ++x) {
            char s = line[x];
            if (s == 'T' || s == '@') continue;     // obstacle
            int id = y * width + x;
            Node* v = new Node(id, x, y);
            V[id] = v;
        }
        ++y;
    }
    if (y != height) error("Mismatch in height");

    // generate edge connections
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!existNode(x, y)) continue;
            Node* v = getNode(x, y);
            if (existNode(x, y + 1)) v->neighbor.push_back(getNode(x, y + 1));
            if (existNode(x - 1, y)) v->neighbor.push_back(getNode(x - 1, y));
            if (existNode(x, y - 1)) v->neighbor.push_back(getNode(x, y - 1));
            if (existNode(x + 1, y)) v->neighbor.push_back(getNode(x + 1, y));
        }
    }
    file.close();

    if (load_weights) {
        auto wid = [&](int x, int y, int ch) {
            return (y * width + x) * channels + ch;
        };

        file.open(map_file + ".weights");
        if (!file) {
            warn("File " + map_file + ".weights not found; Assuming uniform undirected connections");
            channels = 4;
            weights.resize(height * width * channels, MAX_WEIGHT);
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    if (!existNode(x, y)) continue;
                    if (existNode(x, y + 1)) weights[wid(x, y, 0)] = 1.f;
                    if (existNode(x - 1, y)) weights[wid(x, y, 1)] = 1.f;
                    if (existNode(x, y - 1)) weights[wid(x, y, 2)] = 1.f;
                    if (existNode(x + 1, y)) weights[wid(x, y, 3)] = 1.f;
                }
            }
            // write weights file
            std::ofstream myfile(map_file + ".weights");
            myfile << "height " << height << std::endl;
            myfile << "width " << width << std::endl;
            myfile << "channels " << channels << std::endl;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    if (!existNode(x, y)) continue;
                    myfile << x << " " << y;
                    for (int ch = 0; ch < channels; ++ch) {
                        float w = getWeight(x, y, ch);
                        if (w >= MAX_WEIGHT) w = -1.f;      // impassable connections
                        myfile << " " << w;
                    }
                    myfile << std::endl;
                }
            }
            myfile.close();
        } else {
            // read specifications
            while (getline(file, line)) {
                if (*(line.end() - 1) == 0x0d) line.pop_back();
                if (std::regex_match(line, match, r_height)) {
                    if (height != std::stoi(match[1].str())) error ("Incorrect height value");
                    continue;
                }
                if (std::regex_match(line, match, r_width)) {
                    if (width != std::stoi(match[1].str())) error("Incorrect width value");
                    continue;
                }
                if (std::regex_match(line, match, r_channels)) {
                    channels = std::stoi(match[1].str());
                    if (channels != 4) error("Current implementation only support four channels");
                    break;
                }
            }
            weights.resize(height * width * channels, MAX_WEIGHT);
            
            // gather weights
            while (getline(file, line)) {
                if (*(line.end() - 1) == 0x0d) line.pop_back();
                std::stringstream ss(line); std::string val;
                getline(ss, val, ' '); int x = std::stoi(val);
                getline(ss, val, ' '); int y = std::stoi(val);
                if (!existNode(x, y)) continue;
                for (int ch = 0; ch < channels; ++ch) {
                    getline(ss, val, ' ');
                    if (std::stof(val) >= 0.f) {
                        weights[wid(x, y, ch)] = std::stof(val);
                    }
                }
            }
        }
        file.close();
    } else { 
        // no weights
        channels = 0;
        weights.resize(0);
    }
    info("Build graph", t_start);
}

void Grid::setWeights(const std::vector<float>& weights) {
    if (weights.size() % (height * width) != 0) error("Invalid size of weights");
    channels = (int)weights.size() / (height * width);
    if (channels != 4) error("Attempted to set weights with invalid number of channels; Current implementation only support four channels");
    this->weights = weights;
}

float Grid::getWeight(int x, int y, int ch) const {
    return weights.at((y * width + x) * channels + ch);
}

float Grid::getWeight(Node* const u, int ch) const {
    return weights.at(u->id * channels + ch);
}

float Grid::getWeight(Node* const u, Node* const v) const {
    if (!inArray(v, u->neighbor)) error("Nodes u and v are not neighbors");
    if (v->pos == u->pos + Pos(0, 1)) return getWeight(u, 0);
    if (v->pos == u->pos - Pos(1, 0)) return getWeight(u, 1);
    if (v->pos == u->pos - Pos(0, 1)) return getWeight(u, 2);
    if (v->pos == u->pos + Pos(1, 0)) return getWeight(u, 3);
    return MAX_WEIGHT;
}

int Grid::getNeighbor(const State& s, std::array<State, 4>& buf) const {
    int cnt = 0;
    if (s.orientation == -1) {
        for (auto v : s.node->neighbor) {
            buf[cnt++] = State(v);
        }
    } else {
        Pos pos;
        switch (s.orientation) {
            case 0 : pos = s.node->pos + Pos(0, 1); break;
            case 1 : pos = s.node->pos - Pos(1, 0); break;
            case 2 : pos = s.node->pos - Pos(0, 1); break;
            case 3 : pos = s.node->pos + Pos(1, 0); break;
            default : error("Unknown agent orientation");
        }
        if (existNode(pos.x, pos.y)) buf[cnt++] = State(getNode(pos.x, pos.y), s.orientation);
        buf[cnt++] = State(s.node, (s.orientation + 1) % 4);
        buf[cnt++] = State(s.node, (s.orientation + 3) % 4);
    }
    return cnt;
}

bool Grid::existNode(int id) const {
    return 0 <= id && id < height * width && V[id] != nullptr;
}

bool Grid::existNode(int x, int y) const {
    return 0 <= x && x < width && 0 <= y && y < height && existNode(y * width + x);
}

std::pair<Path, float> Grid::getPathWithCost(const State& s, const State& g, std::mt19937* MT, const Nodes& prohibited) const {
    // shortest cost (weight) path
    if (s == g) return std::make_pair(Path(0), 0.f);

    struct AStarNode {
        State state;
        float g;
        float f;
        int parent;
    };

    std::vector<AStarNode> pool;
    pool.reserve(height * width * channels);
    auto compare = [&](int a, int b) {
        if (pool[a].f != pool[b].f) return pool[a].f > pool[b].f;
        if (pool[a].g != pool[b].g) return pool[a].g < pool[b].g;
        return false;
    };
    std::unordered_set<Node*> prohibitedSet(prohibited.begin(), prohibited.end());
    std::priority_queue<int, std::vector<int>, decltype(compare)> OPEN(compare);
    std::unordered_set<State, State::Hasher> CLOSE;

    int last = -1;
    float cost = -1.f;      // cost of path
    pool.push_back({s, 0.f, dist(s.node, g.node), -1});
    OPEN.push(0);
    while (!OPEN.empty()) {
        int idx = OPEN.top(); OPEN.pop();
        AStarNode& curr = pool[idx];
        if (CLOSE.find(curr.state) != CLOSE.end()) continue;
        CLOSE.insert(curr.state);

        if (curr.state == g) {
            last = idx;
            cost = curr.g;
            break;
        }

        std::array<State, 4> buf;
        int cnt = getNeighbor(curr.state, buf);
        if (MT != nullptr) std::shuffle(buf.begin(), buf.begin() + cnt, *MT);
        for (int i = 0; i < cnt; ++i) {
            State next = buf[i];
            if (CLOSE.find(next) != CLOSE.end()) continue;
            if (prohibitedSet.count(next.node)) continue;
            float w = (curr.state.orientation == next.orientation) ? getWeight(curr.state.node, next.node) : 1.f;
            if (w >= MAX_WEIGHT) continue;
            float gcost = curr.g + w;
            float fcost = gcost + dist(next.node, g.node);
            pool.push_back({next, gcost, fcost, idx});
            OPEN.push((int)pool.size() - 1);
        }
    }

    if (last == -1) {
        // no path found
        std::make_pair(Path(0), 0.f);
    }
    Path path;
    for (int i = last; i != -1; i = pool[i].parent) {
        path.push_back(pool[i].state);
    }
    std::reverse(path.begin(), path.end());
    return std::make_pair(path, cost);
}
