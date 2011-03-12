#include <iostream>
#include "IOManager.hh"

using namespace gazebo;

IOManagerPtr IOManager::self;

IOManager::IOManager()
  : work(this->io_service)
{
}

IOManager::~IOManager()
{
  this->Stop();
}

const IOManagerPtr &IOManager::Instance()
{
  if (!self)
  {
    self.reset(new IOManager);
  }

  return self;
}

boost::asio::io_service &IOManager::GetIO()
{
  return this->io_service;
}

void IOManager::Start()
{
  this->thread = boost::thread( boost::bind(&boost::asio::io_service::run, 
                                            &this->io_service) );
}

void IOManager::Stop()
{
  this->thread.interrupt();
  this->io_service.stop();
}
