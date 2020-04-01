#include <commons.h>

class Menu {

public:
  Menu(int begin_y, int begin_x, std::vector<std::string> &text);

  WINDOW *getWin();
  int     getLine();

  std::optional<std::tuple<int, int>> iterate(std::vector<std::string> &text, int key, bool refresh);

private:
  WINDOW *win_;
  int     line = 0;
};
