#include <rclcpp/rclcpp.hpp>
#include <path/msg/data.hpp> 
#include <memory>

struct Point {
    int x, y;        // Координаты узла
    int g;           // Расстояни до начальной вершины
    int h;           // Расстояние до конечной вершины
    Point* parent;    // Родительский узел
};

int distanse(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

class AStar
{
public:
    AStar(std::vector<std::vector<std::string>>& grid, int startX, int startY, int finishX, int finishY)
    : _grid(grid)
    {
        _numCols = grid[0].size(); 
        _numRows = grid.size();
        _points = std::vector<Point>(_numRows * _numCols, {0, 0, 0, 0, nullptr});

        _startPoint = &_points[(startX - 1) * _numRows + startY - 1];
        _finishPoint = &_points[(finishX - 1) * _numRows + finishY - 1];
        _startPoint->x = startX;
        _startPoint->y = startY;

        _finishPoint->x = finishX;
        _finishPoint->y = finishY;

        _openSet.push_back(_startPoint);

        _closedSet = std::vector<Point*>(_numRows * _numCols, nullptr);
    }

    ~AStar()
    {
        _points.clear();
        _openSet.clear();
        _closedSet.clear();
        _grid.clear();
    }

    std::vector<std::vector<std::string>> run(){
        while (!_openSet.empty()) {
        Point* current = _openSet[0];
        int currentIndex = 0;

        for (size_t i = 1; i < _openSet.size(); i++) {
            if (_openSet[i]->g + _openSet[i]->h < current->g + current->h) {
                current = _openSet[i];
                currentIndex = i;
            }
        }

        _openSet.erase(_openSet.begin() + currentIndex);

        if (current == _finishPoint) {
            while (current != nullptr) {
              if (_grid[current->y - 1][current->x - 1] != "S" && _grid[current->y - 1][current->x - 1] != "F")
              _grid[current->y - 1][current->x - 1] = "x";
              current = current->parent;
            }
            return _grid;
        }

        int x = current->x;
        int y = current->y;

        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};

        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];

            if (newX - 1 >= 0 && newX - 1 < _numCols && newY - 1 >= 0 && newY - 1 < _numRows) {
                int newIndex = (newX - 1) * _numRows + newY - 1;

                if (_grid[newY - 1][newX - 1] != "1" && _closedSet[newIndex] == nullptr) {
                    int tentativeG = current->g + 1; 
                    if (_points[newIndex].g == 0 || tentativeG < _points[newIndex].g) {
                        _points[newIndex].x = newX;
                        _points[newIndex].y = newY;
                        _points[newIndex].g = tentativeG;
                        _points[newIndex].h = distanse(newX, newY, _finishPoint->x, _finishPoint->y);
                        _points[newIndex].parent = current;
                        _openSet.push_back(&_points[newIndex]);
                    }
                }
            }
        }

        _closedSet[(x - 1) * _numRows + y - 1] = current;
    }
    return{};
  }

  private:

    int _numRows;
    int _numCols;
    std::vector<Point> _points;
    Point* _startPoint;
    Point* _finishPoint;
    std::vector<Point*> _openSet;
    std::vector<Point*> _closedSet;
    std::vector<std::vector<std::string>>& _grid;
};



class PathNode : public rclcpp::Node {
public:
  PathNode() : Node("path_node") {
    subscription_ = this->create_subscription<path::msg::Data>(
      "numbers", 0, std::bind(&PathNode::Map, this, std::placeholders::_1));
  }
  void Map(const path::msg::Data::SharedPtr msg) {
    for (int i = 0; i < msg->y; i++) {
        std::vector<std::string> row(msg->x, "0");
        map.push_back(row);
    }
    map[msg->start_y - 1][msg->start_x - 1] = "S";
    map[msg->finish_y - 1][msg->finish_x - 1] = "F";
    for (int i = 0; i <= msg->kol_obst * 4 - 4; i+=4){
      for (int j = msg->obstacles[i+1]; j <= msg->obstacles[i+3]; j++){
          for (int l = msg->obstacles[i]; l <= msg->obstacles[i+2]; l++){
              map[j-1][l-1] = "1";
              }
      }
    }
    AStar a_star = AStar(map, msg->start_x, msg->start_y, msg->finish_x, msg->finish_y);
    std::vector<std::vector<std::string>> new_map = a_star.run();
    int m = 0;
    for (int i = 0; i < msg->y; i++) {
        for (int j = 0; j < msg->x; j++) {
            std::cout << new_map[i][j];
            if (new_map[i][j] == "x")
            m = m + 1;
        }
    std::cout << std::endl;
    }
    std::cout << "M=" << m + 1 << std::endl;
};
  rclcpp::Subscription<path::msg::Data>::SharedPtr subscription_;
  std::vector<std::vector<std::string>> map;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
