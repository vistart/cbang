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

#pragma once

#include "Connection.h"
#include "Buffer.h"

#include <limits>
#include <list>
#include <functional>


namespace cb {
  namespace Event {
    class HTTPServer;
    class Request;

    class HTTPConn : public Connection {
    protected:
      unsigned maxBodySize   = std::numeric_limits<int>::max();
      unsigned maxHeaderSize = std::numeric_limits<int>::max();

      Buffer input;

      typedef std::list<SmartPointer<Request> > requests_t;
      requests_t requests;

    public:
      HTTPConn(Base &base);
      virtual ~HTTPConn() {}

      unsigned getMaxBodySize() const {return maxBodySize;}
      void setMaxBodySize(unsigned size) {maxBodySize = size;}

      unsigned getMaxHeaderSize() const {return maxHeaderSize;}
      void setMaxHeaderSize(unsigned size) {maxHeaderSize = size;}

      unsigned getNumRequests() const {return requests.size();}
      typedef requests_t::const_iterator iterator;
      iterator beginRequests() const {return requests.begin();}
      iterator endRequests() const {return requests.end();}

      virtual bool isIncoming() const = 0;
      virtual void writeRequest(const SmartPointer<Request> &req,
                                Buffer buffer, bool hasMore = false,
                                std::function<void (bool)> cb = 0) = 0;
      virtual void makeRequest(const SmartPointer<Request> &req) {}

      void readChunks(const SmartPointer<Request> &req,
                      std::function<void (bool)> cb);

    protected:
      void readChunk(const SmartPointer<Request> &req, uint32_t size,
                     std::function<void (bool)> cb);
      void readChunkTrailer(const SmartPointer<Request> &req,
                            std::function<void (bool)> cb);

      void checkActive(const SmartPointer<Request> &req);
      const SmartPointer<Request> &getRequest();
      void push(const SmartPointer<Request> &req);
      void pop();

    public:
      // From FD
      void close() override;
    };
  }
}
