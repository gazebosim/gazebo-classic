#ifndef CALLBACKHELPER_HH
#define CALLBACKHELPER_HH

#include <boost/function.hpp>

namespace gazebo
{
  class CallbackHelper
  {
    public: CallbackHelper() {}
    public: virtual void OnRead(const std::vector<char> &newdata) = 0;
  };

  template<class M>
  class CallbackHelperT : public CallbackHelper
  {
    public: CallbackHelperT(const boost::function<void (const M &)> &callback) 
            : CallbackHelper(), callback(callback)
            {}

    public: virtual void OnRead(const std::vector<char> &newdata)
            {
              // Extract the data structure from the data
              /*try
              {
                std::string archive_data(&newdata[0], newdata.size());
                std::istringstream archive_stream(archive_data);
                boost::archive::text_iarchive archive(archive_stream);
                archive >> this->data;
              }
              catch (std::exception &e)
              {
                // Unable to decode data
                boost::system::error_code error(boost::asio::error::invalid_argument);
                std::cerr << "!Error:" << e.what() << std::endl;
                return;
              }
              */
              Message::fillFromBuffer(this->data, newdata);
              this->callback( this->data );
            }

    public: boost::function<void (const M &)> callback;
    public: M data;
  };

  typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;
}


#endif
