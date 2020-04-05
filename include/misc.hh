#ifndef _MISC_HH
#define _MISC_HH

#include <cstdio>
#include <string>

#include "Boolean.hh"
#include "ToString.hh"

class Rect : public ToString {
 public:
  Rect() {}
  Rect(unsigned _x, unsigned _y, unsigned _w, unsigned _h)
      : x(_x), y(_y), w(_w), h(_h) {}
  Boolean contains(const Rect& rect) {
    return x <= rect.x && y <= rect.y && x + w >= rect.x + rect.w &&
           y + h >= rect.y + rect.h;
  }
  Boolean contains(const unsigned& _x, const unsigned& _y) {
    return _x >= x && _y >= y && _x <= x + w && _y <= y + h;
  }
  Boolean equals(const Rect& rect) {
    return (rect.x == x && rect.y == y && rect.w == w && rect.h == h);
  }
  virtual string toString() {
    char str[256];
    sprintf(str, "Rect(%u, %u, %u, %u)", x, y, w, h);
    return string(str);
  }
  unsigned x, y, w, h;
};

// A function that outputs a string that identifies each stream (for debugging
// output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env,
                             const RTSPClient& rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for
// debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env,
                             const MediaSubsession& subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

#endif
