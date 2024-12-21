#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>

using namespace std;

// Cau truc Node dai dien cho mot diem tren luoi
struct Node {
    int x, y; // Toa do cua diem
    double g, h, f; // Cac gia tri g, h, f trong thuat toan A*
    Node* parent; // Con tro toi nut cha

    Node(int x, int y, double g = 0, double h = 0, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

    // Toan tu so sanh de su dung trong hang doi uu tien
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// Ham tinh toan gia tri heuristic (khoang cach uoc luong tu diem hien tai den dich)
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// Ham lay cac diem lan can cua mot diem tren luoi
vector<Node*> get_neighbors(Node* node, const vector<vector<int>>& grid) {
    vector<Node*> neighbors;
    int dx[] = { -1, 1, 0, 0 };
    int dy[] = { 0, 0, -1, 1 };

    for (int i = 0; i < 4; ++i) {
        int nx = node->x + dx[i];
        int ny = node->y + dy[i];

        if (nx >= 0 && nx < grid.size() && ny >= 0 && ny < grid[0].size() && grid[nx][ny] == 0) {
            neighbors.push_back(new Node(nx, ny));
        }
    }

    return neighbors;
}

// Ham tai tao duong di tu diem dich ve diem bat dau
vector<Node*> reconstruct_path(Node* node) {
    vector<Node*> path;
    while (node) {
        path.push_back(node);
        node = node->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

// Ham ket hop thuat toan A* voi BFS
vector<Node*> a_star_bfs(const vector<vector<int>>& grid, Node* start, Node* goal) {
    priority_queue<Node*, vector<Node*>, greater<Node*>> open_set; // Hang doi uu tien cho A*
    unordered_map<int, Node*> all_nodes; // Ban do luu tru tat ca cac nut da duyet
    queue<Node*> bfs_queue; // Hang doi cho BFS

    start->h = heuristic(start->x, start->y, goal->x, goal->y);
    start->f = start->g + start->h;
    open_set.push(start);
    bfs_queue.push(start);
    all_nodes[start->x * grid[0].size() + start->y] = start;

    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();

        if (current->x == goal->x && current->y == goal->y) {
            return reconstruct_path(current);
        }

        vector<Node*> neighbors = get_neighbors(current, grid);
        for (Node* neighbor : neighbors) {
            double tentative_g = current->g + 1;

            if (all_nodes.find(neighbor->x * grid[0].size() + neighbor->y) == all_nodes.end() || tentative_g < neighbor->g) {
                neighbor->g = tentative_g;
                neighbor->h = heuristic(neighbor->x, neighbor->y, goal->x, goal->y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                open_set.push(neighbor);
                bfs_queue.push(neighbor);
                all_nodes[neighbor->x * grid[0].size() + neighbor->y] = neighbor;
            }
        }
    }

    return vector<Node*>();
}

int main() {
    vector<vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    Node* start = new Node(0, 0);
    Node* goal = new Node(4, 4);

    vector<Node*> path = a_star_bfs(grid, start, goal);

    if (!path.empty()) {
        cout << "Path found:" << endl;
        for (Node* node : path) {
            cout << "(" << node->x << ", " << node->y << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found." << endl;
    }

    return 0;
}

