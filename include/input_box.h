#include <commons.h>

class InputBox {

public:
  InputBox(int size, WINDOW* win, double initial_value);
  unsigned long Process(int key_in);
  void   Print(int line, bool active);
  double getDouble();

  inline static unsigned long     cursor_;

private:
  WINDOW*           win_;
  unsigned long     size_;
  std::vector<char> buffer_;
};
