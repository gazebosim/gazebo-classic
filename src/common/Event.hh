/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef EVENT_HH
#define EVENT_HH

#include <iostream>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "common/Time.hh"

namespace gazebo
{
  namespace event
  {
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;
    typedef std::vector<ConnectionPtr> Connection_V;

    /// \brief Base class for all events
    class Event
    { 
      public: virtual void Disconnect(ConnectionPtr &c) = 0;
    };

    /// \brief A class that encapsulates a connection
    class Connection
    {
      public: Connection() :event(NULL), id(-1) {}
      public: Connection(Event *e, int i);
      public: ~Connection();
      public: int Id() const;
      private: Event *event;
      private: int id;

      private: static int counter;
      private: int uniqueId;

      private: common::Time creationTime;
      public: template<typename T> friend class EventT;
    };
    
    /// \brief An class for event processing
    template< typename T>
    class EventT : public Event
    {
      /// \brief Connect a callback to this event
      /// \return A Connection object, which will automatically call
      ///         Disconnect when it goes out of scope
      public: ConnectionPtr Connect(const boost::function<T> &subscriber);

      /// \brief Disconnect a callback to this event
      public: virtual void Disconnect(ConnectionPtr &c);

      public: void operator()()
              { this->Signal(); }

      public: void Signal()
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])();
                }
              }

      public: template< typename P >
              void operator()(const P &p)
              { this->Signal(p); }

      public: template< typename P >
              void Signal(const P &p)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p);
                }
              }

      public: template< typename P1, typename P2 >
              void Signal(const P1 &p1, const P2 &p2)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2);
                }
              }

      public: template< typename P1, typename P2, typename P3 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5);
                }
              }


      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5, const P6 &p6)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5, const P6 &p6, const P7 &p7)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5, const P6 &p6, const P7 &p7, const P8 &p8)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7, p8);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5, const P6 &p6, const P7 &p7, const P8 &p8,
                          const P9 &p9)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7, p8, p9);
                }
              }

      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9, typename P10 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4,
                          const P5 &p5, const P6 &p6, const P7 &p7, const P8 &p8,
                          const P9 &p9, const P10 &p10)
              {
                for (unsigned int i=0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                }
              }

      private: std::vector< boost::function<T> *> connections;
    };

    template<typename T>
    ConnectionPtr EventT<T>::Connect(const boost::function<T> &subscriber)
    {
      this->connections.push_back(new boost::function<T>(subscriber) );
      return ConnectionPtr( new Connection(this, this->connections.size()-1) );
    }
    
    template<typename T>
    void EventT<T>::Disconnect(ConnectionPtr &c)
    {
      if (c->Id() >=0 && c->Id() < (int)this->connections.size())
      {
        this->connections.erase(this->connections.begin()+c->Id());
        c->event = NULL;
        c->id = -1;
      }
    }
  }
}
#endif
