#ifndef CALLBACKHELPER_HH
#define CALLBACKHELPER_HH

#include <vector>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <google/protobuf/message.h>

#include "common/Console.hh"
#include "msgs/msgs.h"
#include "common/Exception.hh"

namespace gazebo
{
  namespace transport
  {
    class CallbackHelper
    {
      public: CallbackHelper() {}

      /// \brief Get the typename of the message that is handled
      public: virtual std::string GetMsgType() const = 0;

      public: virtual bool HandleMessage(const google::protobuf::Message *msg) = 0;
      public: virtual bool HandleData(const std::string &newdata) = 0;

      /// \brief Return true if the callback is local, false if the callback
      ///        is tied to a remote connection
      public: virtual bool IsLocal() const = 0;
    };
    typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;


    template<class M>
    class CallbackHelperT : public CallbackHelper
    {
      public: CallbackHelperT( const boost::function<void (const boost::shared_ptr<M const> &)> &cb) : callback(cb) 
              {
                // Just some code to make sure we have a google protobuf.
                /*M test;
                google::protobuf::Message *m;
                if ( (m=dynamic_cast<google::protobuf::Message*>(&test)) ==NULL)
                  gzthrow( "Message type must be a google::protobuf type\n" );
                  */
              }

      /// \brief Get the typename of the message that is handled
      public: std::string GetMsgType() const
              {
                M test;
                google::protobuf::Message *m;
                if ((m=dynamic_cast<google::protobuf::Message*>(&test)) ==NULL)
                  gzthrow( "Message type must be a google::protobuf type\n" );
                return m->GetTypeName();
              }

      public: virtual bool HandleMessage(const google::protobuf::Message *msg)
              {
                /*boost::shared_ptr<M> m( new M );
                m->ParseFromString( ((msgs::Packet*)msg)->serialized_data() );
                */
                boost::shared_ptr<M> m( new M );
                m->CopyFrom(*msg);

                this->callback( m );
                return true;
              }

      public: virtual bool HandleData(const std::string &newdata)
              {
                /*msgs::Packet packet;
                packet.ParseFromString(newdata);

                // TODO: Handle this error properly
                if (packet.type() != "data")
                  gzerr << "CallbackHelperT::HandleMessage Invalid message!!!\n";
                boost::shared_ptr<M> m( new M );
                m->ParseFromString( packet.serialized_data() );
                */

                boost::shared_ptr<M> m( new M );
                m->ParseFromString( newdata );
                this->callback( m );
                return true;
              }

      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (const boost::shared_ptr<M const> &)> callback;
    };

    class DebugCallbackHelper : public CallbackHelper
    {
      public: DebugCallbackHelper( const boost::function<void (const boost::shared_ptr<msgs::String const> &)> &cb) : callback(cb) 
              {
              }

      /// \brief Get the typename of the message that is handled
      public: std::string GetMsgType() const
              {
                msgs::String m;
                return m.GetTypeName();
              }

      public: virtual bool HandleData(const std::string &newdata)
              {
                msgs::Packet packet;
                packet.ParseFromString(newdata);

                // TODO: Handle this error properly
                if (packet.type() != "data")
                  gzerr << "CallbackHelperT::HandleData Invalid message!!!\n";
                boost::shared_ptr<msgs::String> m( new msgs::String );
                m->ParseFromString( packet.serialized_data() );
                this->callback( m );
                return true;
              }

      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (const boost::shared_ptr<msgs::String const> &)> callback;
    };
  }
}

#endif
