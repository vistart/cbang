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

#include "OutgoingRequest.h"

#include <cbang/SmartPointer.h>
#include <cbang/util/RateSet.h>
#include <cbang/openssl/SSLContext.h>

#include <map>
#include <functional>


namespace cb {
  class URI;

  namespace Event {
    class Base;
    class DNSBase;

    class Client {
      Base &base;
      DNSBase &dns;
      SmartPointer<SSLContext> sslCtx;
      IPAddress bindAddr;
      unsigned readTimeout  = 0;
      unsigned writeTimeout = 0;
      SmartPointer<RateSet> stats;

    public:
      typedef SmartPointer<OutgoingRequest> RequestPtr;

      template <class T> struct Callback {
        typedef void (T::*member_t)(Request &);
      };

      typedef OutgoingRequest::callback_t callback_t;

      Client(Base &base, DNSBase &dns,
             const SmartPointer<SSLContext> &sslCtx = 0);
      ~Client();

      Base &getBase() const {return base;}
      DNSBase &getDNS() const {return dns;}

      const cb::SmartPointer<SSLContext> &getSSLContext() const {return sslCtx;}
      void setSSLContext(const cb::SmartPointer<SSLContext> &sslCtx)
        {this->sslCtx = sslCtx;}

      const IPAddress &getBindAddress() const {return bindAddr;}
      void setBindAddress(const IPAddress &bind) {this->bindAddr = bind;}

      unsigned getReadTimeout() const {return readTimeout;}
      void setReadTimeout(unsigned timeout) {readTimeout = timeout;}

      unsigned getWriteTimeout() const {return writeTimeout;}
      void setWriteTimeout(unsigned timeout) {writeTimeout = timeout;}

      const SmartPointer<RateSet> &getStats() const {return stats;}
      void setStats(const SmartPointer<RateSet> &stats) {this->stats = stats;}

      void send(const SmartPointer<Request> &req) const;

      RequestPtr call(const URI &uri, RequestMethod method, const char *data,
                      unsigned length, callback_t cb);

      RequestPtr call(const URI &uri, RequestMethod method,
                      const std::string &data, callback_t cb);

      RequestPtr call(const URI &uri, RequestMethod method, callback_t cb);


      // Member callbacks
      template <class T>
      callback_t bind(T *obj, typename Callback<T>::member_t member) {
        return std::bind(member, obj, std::placeholders::_1);
      }

      template <class T> RequestPtr
      call(const URI &uri, RequestMethod method, const char *data,
           unsigned length, T *obj, typename Callback<T>::member_t member)
      {return call(uri, method, data, length, bind(obj, member));}

      template <class T> RequestPtr
      call(const URI &uri, RequestMethod method, const std::string &data,
           T *obj, typename Callback<T>::member_t member)
      {return call(uri, method, data, bind(obj, member));}

      template <class T> RequestPtr
      call(const URI &uri, RequestMethod method,
           T *obj, typename Callback<T>::member_t member)
      {return call(uri, method, bind(obj, member));}
    };
  }
}
