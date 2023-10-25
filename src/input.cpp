#include <rclcpp/rclcpp.hpp>
#include <path/msg/data.hpp> 
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("input_node");
  auto publisher = node->create_publisher<path::msg::Data>("numbers", 10);
  auto msg = std::make_shared<path::msg::Data>(); 

  while (rclcpp::ok()) {
    auto inp = path::msg::Data();
            int x, y, k, x1, y1, x2, y2;
            std::vector<int> obstacles;
            std::vector<std::vector<std::string>> map = {};
            std::cout << "Введите x=";
            std::cin >> x;
            inp.x = x;
            std::cout << "Введите y=";
            std::cin >> y;
            inp.y = y;
            for (int i = 0; i < y; i++) {
                std::vector<std::string> row(x, "0");
                map.push_back(row);
            }
            std::cout << "Введите координаты старта через пробел:";
            std::cin >> x >> y;
            inp.start_x = x;
            inp.start_y = y;
            map[y - 1][x - 1] = "S";
            std::cout << "Введите координаты финиша через пробел:";
            std::cin >> x >> y;
            inp.finish_x = x;
            inp.finish_y = y;
            map[y - 1][x - 1] = "F";
            std::cout << "Введите количество препятствий:";
            std::cin >> k;
            inp.kol_obst = k;
            for (int i = 1; i <= k; i++) {
                std::cout << "Введите координаты левого верхнего угла " << i << " препятствия: ";
                std::cin >> x1 >> y1;
                obstacles.push_back(x1);
                obstacles.push_back(y1);
                std::cout << "Введите координаты правого нижнего угла " << i << " препятствия: ";
                std::cin >> x2 >> y2;
                obstacles.push_back(x2);
                obstacles.push_back(y2);
                inp.obstacles = obstacles;
                for (int j = y1; j <= y2; j++){
                    for (int l = x1; l <= x2; l++){
                        map[j - 1][l - 1] = "1";
                    }
                }
            }
            std::cout << std::endl;
            std::cout << "Размер поля " << inp.x << " " << inp.y << std::endl;
            std::cout << "Точка старта "<< inp.start_x << " " << inp.start_y << std::endl;
            std::cout << "Точка финиша "<< inp.finish_x << " " << inp.finish_y << std::endl;
            std::cout << "Количество препятствий " << inp.kol_obst << std::endl;
            for (int i = 0; i <= k * 4 - 4; i+=4) {
            std::cout << "{" << obstacles[i] << ", " << obstacles[i+1] << "} {" << obstacles[i+2] << ", " << obstacles[i+3] << "}"<< std::endl;
            }
            for (int i = 0; i < inp.y; i++) {
                for (int j = 0; j < inp.x; j++) {
                    std::cout << map[i][j];
                }
                std::cout << std::endl;
            }
    publisher->publish(inp);
  }

  rclcpp::shutdown();
  return 0;
}

