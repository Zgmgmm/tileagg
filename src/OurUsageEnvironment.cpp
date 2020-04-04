/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 3 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// Copyright (c) 1996-2020 Live Networks, Inc.  All rights reserved.
// My Usage Environment: for a simple, non-scripted, console application
// Implementation

#include "OurUsageEnvironment.hh"

#include <glog/logging.h>

#include <iostream>

////////// OurUsageEnvironment //////////

#if defined(__WIN32__) || defined(_WIN32)
extern "C" int initializeWinsockIfNecessary();
#endif

using namespace google;

OurUsageEnvironment::OurUsageEnvironment(TaskScheduler& taskScheduler)
    : BasicUsageEnvironment0(taskScheduler) {
  fLogPtr = fLogBuffer;
#if defined(__WIN32__) || defined(_WIN32)
  if (!initializeWinsockIfNecessary()) {
    setResultErrMsg("Failed to initialize 'winsock': ");
    reportBackgroundError();
    internalError();
  }
#endif
}

OurUsageEnvironment::~OurUsageEnvironment() {}

OurUsageEnvironment* OurUsageEnvironment::createNew(
    TaskScheduler& taskScheduler) {
  return new OurUsageEnvironment(taskScheduler);
}

int OurUsageEnvironment::getErrno() const {
#if defined(__WIN32__) || defined(_WIN32) || defined(_WIN32_WCE)
  return WSAGetLastError();
#else
  return errno;
#endif
}

UsageEnvironment& OurUsageEnvironment::operator<<(char const* str) {
  if (str == NULL) str = "(NULL)";  // sanity check
  do {
    auto lf = strchr(str, '\n');
    if (lf == NULL) {
      auto write = sprintf(fLogPtr, "%s", str);
      fLogPtr += write;
      str += write;
    } else {  // flush on LF
      snprintf(fLogPtr, lf - str + 1, "%s", str);
      VLOG(INFO) << fLogBuffer;
      fLogPtr = fLogBuffer;
      str = lf + 1;
    }
  } while (strlen(str) != 0);

  return *this;
}

UsageEnvironment& OurUsageEnvironment::operator<<(int i) {
  fLogPtr += sprintf(fLogPtr, "%i", i);
  return *this;
}

UsageEnvironment& OurUsageEnvironment::operator<<(unsigned u) {
  fLogPtr += sprintf(fLogPtr, "%u", u);
  return *this;
}

UsageEnvironment& OurUsageEnvironment::operator<<(double d) {
  fLogPtr += sprintf(fLogPtr, "%lf", d);
  return *this;
}


UsageEnvironment& OurUsageEnvironment::operator<<(void* p) {
  fLogPtr += sprintf(fLogPtr, "%p", p);
  return *this;
}
