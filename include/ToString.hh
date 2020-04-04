

#ifndef _TO_STRING_HH
#define _TO_STRING_HH

#include <string>

using namespace std;

class ToString {
 public:
  virtual string toString() = 0;
};

ostream& operator<<(ostream& out, ToString& o) {
  out << o.toString();
  return out;
}

#endif