#include <commons.h>

class Menu {

public:
  Menu(int begin_y, int begin_x, std::vector<std::string> &text);

  std::optional<std::tuple<int, int>> Iterate(std::vector<std::string> &text, int key);

private:
  WINDOW *win_;
  int     i = 0;
};
