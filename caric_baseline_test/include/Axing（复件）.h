

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;



/* 定义障碍物范围结构体
struct Obstacle {
    Eigen::Vector3d start;
    Eigen::Vector3d end;

    Obstacle(Eigen::Vector3d _start, Eigen::Vector3d _end) : start(_start), end(_end) {}
};*/

struct Obstacle {
    Eigen::Vector3d center;
    Eigen::Vector3d start;
    Eigen::Vector3d end;
    Obstacle(Eigen::Vector3d _center) : center(_center) {
        start = center - Eigen::Vector3d(3.5,3.5,3.5);
        end = center + Eigen::Vector3d(3.5,3.5,3.5);
    }
};

// 定义A*算法中的节点结构体
struct Node {
    Eigen::Vector3d point;
    //std::weak_ptr<Node> parent;
    Node* parent;
    double g;//累积代价
    double h;//启发代价
    double f;//总代价
    Node(Eigen::Vector3d _point, Node* _parent, double _g, double _h) : point(_point), parent(_parent), g(_g), h(_h) {
        f = g + h;
    }
};

// 计算两个三维坐标点之间的曼哈顿距离
double manhattanDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
    return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y()) + abs(p1.z() - p2.z());
    //return (p1-p2).norm();
}

// 判断一个点是否在障碍物范围内
bool isPointInObstacle(Eigen::Vector3d point, vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (point.x() >= obstacle.start.x() && point.x() <= obstacle.end.x() &&
            point.y() >= obstacle.start.y() && point.y() <= obstacle.end.y() &&
            point.z() >= obstacle.start.z() && point.z() <= obstacle.end.z()) {
            return true;
        }
    }
    return false;
}

/* 判断一个点是否在空间范围内
bool isPointInSpace(Eigen::Vector3d point, Eigen::Vector3d spaceSize) {
    return point.x >= 0 && point.x < spaceSize.x &&
           point.y >= 0 && point.y < spaceSize.y &&
           point.z >= 0 && point.z < spaceSize.z;
}*/

// 判断一个点是否为终点
bool isPointGoal(Eigen::Vector3d point, Eigen::Vector3d goal) {
    if((point-goal).norm()<6)
    {
        return true;
    }
    else
    {
        return false;
    }
    //return point.x() == goal.x() && point.y() == goal.y() && point.z() == goal.z();
}

// 获取一个点的相邻点
vector<Eigen::Vector3d> getNeighbors(Eigen::Vector3d point, vector<Obstacle>& obstacles) {
    vector<Eigen::Vector3d> neighbors;
    for (int dx = -3; dx <= 3; dx = dx+1) {
        for (int dy = -3; dy <= 3; dy = dy+1) {
            for (int dz = -3; dz <= 3; dz = dz+1) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                Eigen::Vector3d neighbor(point.x() + dx, point.y() + dy, point.z() + dz);
                if (!isPointInObstacle(neighbor, obstacles)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }
    if(neighbors.size()==0)
    {
        cout << "A*堵死"<<endl;
    }
    return neighbors;
}

bool compareNodes(const Node* a, const Node* b) {
    return a->f > b->f;
}

// A*算法
vector<Eigen::Vector3d> AStar(Eigen::Vector3d start, Eigen::Vector3d goal, vector<Obstacle>& obstacles) {
    vector<Eigen::Vector3d> path;
    //priority_queue<Node*, vector<Node*>, function<bool(Node*, Node*)>> openList([](Node* n1, Node* n2) { return n1->f > n2->f; });
    std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> openList(compareNodes);
    vector<Node*> closedList;
    Node* startNode = new Node(start, nullptr, 0, manhattanDistance(start, goal));
    openList.push(startNode);
    //cout << "启动" << endl;

    while (!openList.empty()) {
        Node* currentNode = openList.top();
        openList.pop();
        //cout << "xiba" << endl;

        if (isPointGoal(currentNode->point, goal)) {
            // 找到路径，回溯构建路径
            Node* node = currentNode;
            while (node != nullptr) {
                path.push_back(node->point);
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            break;
        }

        closedList.push_back(currentNode);
        cout << "找邻居" <<endl;

        vector<Eigen::Vector3d> neighbors = getNeighbors(currentNode->point, obstacles);
        for (Eigen::Vector3d neighbor : neighbors) {
            double g = currentNode->g + manhattanDistance(neighbor,currentNode->point);
            double h = manhattanDistance(neighbor, goal);
            double f = g + h;

            Node* neighborNode = new Node(neighbor, currentNode, g, h);

            bool skipNeighbor = false;
            for (Node* node : closedList) {
                if (node->point.x() == neighborNode->point.x() &&
                    node->point.y() == neighborNode->point.y() &&
                    node->point.z() == neighborNode->point.z() &&
                    node->f <= f) {
                    skipNeighbor = true;
                    break;
                }
            }

            if (skipNeighbor) {
                continue;
            }

            bool inOpenList = false;
            std::vector<Node*> tempOpenList(openList.size());
            std::copy(&openList.top(), &openList.top() + openList.size(), tempOpenList.begin());

            for (Node* node : tempOpenList) {
                if (node->point.x() == neighborNode->point.x() &&
                    node->point.y() == neighborNode->point.y() &&
                    node->point.z() == neighborNode->point.z() &&
                    node->f <= f) {
                    inOpenList = true;
                    break;
                }
            }

            if (!inOpenList) {
                openList.push(neighborNode);
            }
        }
    }

    if (path.empty()) {
        cout << "没有找到有效路径" << endl;
        //return vector<Eigen::Vector3d>();
    }

    // 释放内存
    for (Node* node : closedList) {
        delete node; 
    }
    for (Node* node : openList) {
        delete node; 
    }
    
    cout << "成功A*" <<endl;


    return path;
}
/*使用实例
int main() {
    Eigen::Vector3d start(0, 0, 0);
    Eigen::Vector3d goal(4, 4, 4);
    
    vector<Obstacle> obstacles = { Obstacle(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)) };

    vector<Eigen::Vector3d> path = AStar(start, goal, obstacles);

    if (path.empty()) {
        cout << "No path found!" << endl;
    } else {
        cout << "Path found: ";
        for (const auto& point : path) {
            cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
        }
        cout << endl;
    }

    return 0;
}*/