#include <commons.h>

class InputBox {

public:
  InputBox(int size, WINDOW* win);
  void   Process(int key_in);
  void   Print();
  double getDouble();

private:
  WINDOW*           win_;
  unsigned long     size_;
  unsigned long     cursor_;
  std::vector<char> buffer_;
};
