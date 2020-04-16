#ifndef _BLOCKING_QUEUE_H
#define _BLOCKING_QUEUE_H

#include <condition_variable>
#include <deque>
#include <mutex>

template <typename T>
class BlockingQueue {
 public:
  BlockingQueue(u_int32_t maxSize = 102400) : fMaxSize(maxSize) {}

  void push(const T& item, long timeout = 0) {
    {
      std::unique_lock<std::mutex> lck(fMtx);
      while (fQue.size() >= fMaxSize) {
        fCondNotFull.wait(lck);
      }
      fQue.push_back(item);
    }
    fCondNotEmpty.notify_all();
  }

  T pop(long timeout = 0) {
    static int cnt = 0;
    T item;
    {
      cnt++;
      std::unique_lock<std::mutex> lck(fMtx);
      while (fQue.empty()) {
        fCondNotEmpty.wait(lck);
      }
      item = fQue.front();
      fQue.pop_front();
    }
    fCondNotFull.notify_all();
    return item;
  }

  bool empty() { return fQue.empty(); }
  unsigned size() { return fQue.size(); }

 private:
  u_int32_t fMaxSize;
  std::deque<T> fQue;
  std::mutex fMtx;
  std::condition_variable fCondNotFull, fCondNotEmpty;
};

#endif