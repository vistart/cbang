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

#include "IPAddress.h"

#include <cbang/Exception.h>
#include <cbang/String.h>
#include <cbang/socket/Winsock.h>
#include <cbang/log/Logger.h>
#include <cbang/socket/Socket.h>
#include <cbang/os/SysError.h>

#include <cstring>

#ifdef _WIN32
#include <ws2tcpip.h>
#include <wspiapi.h>

#else
#include <sys/types.h>
#include <sys/socket.h>
#  ifdef __FreeBSD__
#include <netinet/in.h>
#  endif
#include <netdb.h>
#endif

using namespace std;
using namespace cb;


IPAddress::IPAddress(uint32_t ip, const std::string &host, uint16_t port) :
  host(host), ip(ip), port(port ? port : portFromString(host)) {
  Socket::initialize(); // Windows needs this

  size_t ptr = host.find(":");
  if (ptr != string::npos) this->host = host.substr(0, ptr);

  if (this->host == "0") this->host = "0.0.0.0"; // OSX & Windows need this
}


IPAddress::IPAddress(uint32_t ip, uint16_t port) :
  host(ipToString(ip)), ip(ip), port(port) {}


IPAddress::IPAddress(const string &host, uint16_t port) :
  IPAddress(ipFromString(host), host, port) {}


bool IPAddress::hasHost() const {
  return host.find_first_not_of("1234567890. \t\n\r") != string::npos;
}


void IPAddress::lookupHost() {host = hostFromIP(*this);}


string IPAddress::toString() const {
  return getHost() + (getPort() ? String::printf(":%u", getPort()) : string());
}


string IPAddress::ipToString(uint32_t ip) {
  return String::printf("%u.%u.%u.%u", ip >> 24, (ip >> 16) & 0xff,
                        (ip >> 8) & 0xff, ip & 0xff);
}


uint32_t IPAddress::ipFromString(const string &host) {
  vector<IPAddress> addrs;

  if (!ipsFromString(host, addrs, 0, 1))
    THROW("Could not get IP address for " << host);

  return addrs[0].getIP();
}


unsigned IPAddress::ipsFromString(const string &host, vector<IPAddress> &addrs,
                                  uint16_t port, unsigned max) {
  // TODO support IPv6 addresses

  Socket::initialize(); // Windows needs this

  string hostname;
  size_t ptr = host.find_last_of(":");
  if (ptr != string::npos) {
    hostname = host.substr(0, ptr);
    port = String::parseU16(host.substr(ptr + 1));

  } else hostname = host;

  if (hostname == "0") hostname = "0.0.0.0";

  struct addrinfo hints;
  struct addrinfo *res = 0;
  int err;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = PF_INET; // Force IPv4 for now
  hints.ai_socktype = SOCK_STREAM;

  if ((err = getaddrinfo(hostname.c_str(), 0, &hints, &res)))
    THROW("Could not get IP address for " << hostname << ": "
           << gai_strerror(err));

  unsigned count = 0;
  struct addrinfo *info;
  for (info = res; info && (!max || count < max); info = info->ai_next) {
    if (!info->ai_addr) continue;
    uint32_t ip = ntohl(((struct sockaddr_in *)info->ai_addr)->sin_addr.s_addr);
    IPAddress addr(ip, port);
    addr.host = hostname;
    addrs.push_back(addr);
    count++;
  }

  freeaddrinfo(res);

  return count;
}


string IPAddress::hostFromIP(const IPAddress &ip) {
  if (ip.hasHost()) return ip.getHost();

  // Force 127.0.0.1 to localhost
  if (ip.getIP() == 0x7f000001) return "localhost";

  Socket::initialize(); // Windows needs this

  struct sockaddr_in addr;
  char buffer[1024];

  // TODO support IPv6
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(ip.getPort());
  addr.sin_addr.s_addr = htonl(ip.getIP());

  if (getnameinfo((struct sockaddr *)&addr, sizeof(addr), buffer,
                  sizeof(buffer), 0, 0, 0))
    THROW("Reverse lookup for " << ip << " failed: " << SysError());

  return buffer;
}


uint16_t IPAddress::portFromString(const string &host) {
  size_t ptr = host.find_last_of(":");
  if (ptr == string::npos) return 0;
  return String::parseU16(host.substr(ptr + 1));
}
