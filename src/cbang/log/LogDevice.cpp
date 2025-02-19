/******************************************************************************\

          This file is part of the C! library.  A.K.A the cbang library.

                Copyright (c) 2003-2023, Cauldron Development LLC
                               All rights reserved.

         The C! library is free software: you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public License
       as published by the Free Software Foundation, either version 2.1 of
               the License, or (at your option) any later version.

        The C! library is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
                 Lesser General Public License for more details.

         You should have received a copy of the GNU Lesser General Public
                 License along with the C! library.  If not, see
                         <http://www.gnu.org/licenses/>.

        In addition, BSD licensing may be granted on a case by case basis
        by written permission from at least one of the copyright holders.
           You may request written permission by emailing the authors.

                  For information regarding this software email:
                                 Joseph Coffland
                          joseph@cauldrondevelopment.com

\******************************************************************************/

#include "LogDevice.h"

#include "Logger.h"

using namespace std;
using namespace cb;


LogDevice::impl::impl(const std::string &prefix, const std::string &suffix,
                      const std::string &trailer, const std::string &rateKey) :
  prefix(prefix), suffix(suffix), trailer(trailer), rateKey(rateKey) {}


LogDevice::impl::~impl() {
  write(&trailer[0], trailer.size());
  flushLine();
}


streamsize LogDevice::impl::write(const char_type *s, streamsize n) {
  std::streamsize total = n;

  while (n) {
    if (startOfLine) {
      // Add prefix
      buffer.append(prefix.begin(), prefix.end());
      startOfLine = false;
    }

    // Copy up to EOL to buffer, skipping '\r's
    std::streamsize i;
    for (i = 0; i < n; i++)
      if (s[i] == '\n') break;
      else if (s[i] != '\r') {
        buffer.append(1, s[i]);
        if (first && !rateKey.empty()) rateMessage.append(1, s[i]);
      }

    if (i == n) break; // No EOL

    // Write line to log
    flushLine();

    // Skip EOL
    i++;

    // Advance
    n -= i;
    s += i;
  }

  return total;
}


void LogDevice::impl::flushLine() {
  if (startOfLine) return;

  if (first && !rateKey.empty() && !rateMessage.empty()) {
    lock();
    Logger::instance().rateMessage(rateKey, rateMessage);
  }
  first = false;

  // Add suffix
  buffer.append(suffix.begin(), suffix.end());

  // Add EOL
  if (Logger::instance().getLogCRLF()) buffer.append(1, '\r');
  buffer.append(1, '\n');

  flush();

  unlock();
  startOfLine = true;
}


bool LogDevice::impl::flush() {
  if (buffer.empty()) return true;

  lock();

  // Write to log
  Logger &logger = Logger::instance();
  logger.write(buffer);
  logger.flush();

  // Flush the buffer
  buffer.clear();

  return true;
}


void LogDevice::impl::lock() {
  if (locked) return;
  Logger::instance().lock();
  locked = true;
}


void LogDevice::impl::unlock() {
  if (!locked) return;
  Logger::instance().unlock();
  locked = false;
}
