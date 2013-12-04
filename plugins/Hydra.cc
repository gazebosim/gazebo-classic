/*********************************************************************
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 *
 **********************************************************************/

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <cstring>

#include "gazebo/physics/physics.hh"

#include "Hydra.hh"

// loosely adapted from the following
// https://github.com/rpavlik/vrpn/blob/razer-hydra/vrpn_Tracker_RazerHydra.C

// and with reference to
// http://lxr.free-electrons.com/source/samples/hidraw/hid-example.c


// Ugly hack to work around failing compilation on systems that don't
// yet populate new version of hidraw.h to userspace.
//
// If you need this, please have your distro update the kernel headers.

#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

// eventually crawl hidraw file system using this:
// http://www.signal11.us/oss/udev/

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(RazerHydra)

/////////////////////////////////////////////////
RazerHydra::RazerHydra()
: hidrawFd(0)
{
  this->stop = false;
  this->lastCycleStart = common::Time::GetWallTime();

  // magic number for 50% mix at each step
  this->periodEstimate.SetFc(0.11, 1.0);

  this->periodEstimate.SetValue(0.004);
}

/////////////////////////////////////////////////
RazerHydra::~RazerHydra()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  this->stop = true;
  this->pollThread->join();
}

/////////////////////////////////////////////////
void RazerHydra::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  int res;
  uint8_t buf[256];
  struct hidraw_report_descriptor rptDesc;
  struct hidraw_devinfo info;

  std::string device = _sdf->Get<std::string>("device");

  this->hidrawFd = open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (this->hidrawFd < 0)
  {
    gzerr << "couldn't open hidraw device[" << device << "]\n";
    return;
  }

  memset(&rptDesc, 0x0, sizeof(rptDesc));
  memset(&info, 0x0, sizeof(info));
  memset(buf, 0x0, sizeof(buf));

  // Get Raw Name
  res = ioctl(this->hidrawFd, HIDIOCGRAWNAME(256), buf);
  if (res < 0)
    perror("HIDIOCGRAWNAME");

  // set feature to start it streaming
  memset(buf, 0x0, sizeof(buf));
  buf[6] = 1;
  buf[8] = 4;
  buf[9] = 3;
  buf[89] = 6;

  int attempt = 0;
  for (attempt = 0; attempt < 50; ++attempt)
  {
    res = ioctl(this->hidrawFd, HIDIOCSFEATURE(91), buf);
    if (res < 0)
    {
      gzerr << "unable to start streaming\n";
      perror("HIDIOCSFEATURE");
      common::Time::MSleep(500);
    }
    else
    {
      break;
    }
  }

  if (attempt >= 60)
  {
    gzerr << "Failed to load hydra\n";
    return;
  }



    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RazerHydra::Update, this, _1));

  this->rightModel = _world->GetModel("right_arm_goal");
  this->leftModel = _world->GetModel("left_arm_goal");

  this->basePoseRight = this->rightModel->GetWorldPose();
  this->basePoseLeft = this->leftModel->GetWorldPose();

  this->pollThread = new boost::thread(boost::bind(&RazerHydra::Run, this));

  this->prevState = this->buttons[0];
}

/////////////////////////////////////////////////
void RazerHydra::Update(const common::UpdateInfo & /*_info*/)
{
  /*common::Time pollTime(1, 50000000);
  double cornerHz = 2.5;

  uint8_t state = this->buttons[0];

  this->Poll(pollTime, cornerHz);
  */

  boost::mutex::scoped_lock lock(this->mutex);
  math::Pose origRight(this->pos[0], this->quat[0]);

  math::Pose pivotRight = origRight;
  math::Pose grabRight = origRight;

  pivotRight.pos += origRight.rot * math::Vector3(0.04, 0, 0);
  grabRight.pos += origRight.rot * math::Vector3(0.12, 0, 0);

  math::Pose origLeft(this->pos[1], this->quat[1]);

  math::Pose pivotLeft = origLeft;
  math::Pose grabLeft = origLeft;

  pivotLeft.pos += origLeft.rot * math::Vector3(0.04, 0, 0);
  grabLeft.pos += origLeft.rot * math::Vector3(0.12, 0, 0);

  if (this->buttons[0])
  {
    if (this->prevState != this->buttons[0])
    {
      this->resetPoseRight = grabRight;
      this->resetPoseLeft = grabLeft;
    }

   this->rightModel->SetWorldPose(
        math::Pose(grabRight.pos - this->resetPoseRight.pos,
          grabRight.rot * this->resetPoseRight.rot.GetInverse()) + this->basePoseRight);

    this->leftModel->SetWorldPose(
        math::Pose(grabLeft.pos - this->resetPoseLeft.pos,
          grabLeft.rot * this->resetPoseLeft.rot.GetInverse()) + this->basePoseLeft);
  }
  else
  {
    this->rightModel->SetWorldPose(this->basePoseRight);
    this->leftModel->SetWorldPose(this->basePoseLeft);
  }

  this->prevState = this->buttons[0];
}

/////////////////////////////////////////////////
void RazerHydra::Run()
{
  common::Time pollTime(0, 5000);
  double cornerHz = 2.5;

  while(!this->stop)
  {
    this->Poll(pollTime, cornerHz);
  }

  if (this->hidrawFd >= 0)
  {
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[89] = 5;
    int res = ioctl(this->hidrawFd, HIDIOCSFEATURE(91), buf);

    if (res < 0)
    {
      gzerr << "unable to stop streaming\n";
      perror("HIDIOCSFEATURE");
    }

    close(this->hidrawFd);
  }
}

/////////////////////////////////////////////////
bool RazerHydra::Poll(const common::Time &_timeToWait, float _lowPassCornerHz)
{
  if (this->hidrawFd < 0)
  {
    gzerr << "hidraw device is not open, couldn't poll.\n";
    return false;
  }

  if(_timeToWait == common::Time::Zero)
  {
    gzerr << "_msToWait must be at least 1.\n";
    return false;
  }

  if(_lowPassCornerHz <= std::numeric_limits<float>::epsilon())
  {
    gzerr << "Corner frequency for low-pass filter must be greater than 0."
      << "Aborting.\n";
    return false;
  }

  common::Time deadline = common::Time::GetWallTime() + _timeToWait;

  uint8_t buf[64];
  //while (common::Time::GetWallTime() < deadline)
  //{
    ssize_t nread = read(this->hidrawFd, buf, sizeof(buf));

    if (nread > 0)
    {
      static bool firstTime = true;

      // Update average read period
      if(!firstTime)
      {
        this->periodEstimate.Process(
            (common::Time::GetWallTime() - this->lastCycleStart).Double());
      }

      this->lastCycleStart = common::Time::GetWallTime();

      if (firstTime)
        firstTime = false;

      // Update filter frequencies
      float fs = 1.0 / this->periodEstimate.GetValue();
      float fc = _lowPassCornerHz;

      for (int i = 0; i < 2; ++i)
      {
        this->filterPos[i].SetFc(fc, fs);
        this->filterQuat[i].SetFc(fc, fs);
      }

      // Read data
      this->rawPos[0] = *((int16_t *)(buf+8));
      this->rawPos[1] = *((int16_t *)(buf+10));
      this->rawPos[2] = *((int16_t *)(buf+12));
      this->rawQuat[0] = *((int16_t *)(buf+14));
      this->rawQuat[1] = *((int16_t *)(buf+16));
      this->rawQuat[2] = *((int16_t *)(buf+18));
      this->rawQuat[3] = *((int16_t *)(buf+20));
      this->rawButtons[0] = buf[22] & 0x7f;
      this->rawAnalog[0] = *((int16_t *)(buf+23));
      this->rawAnalog[1] = *((int16_t *)(buf+25));
      this->rawAnalog[2] = buf[27];

      this->rawPos[3] = *((int16_t *)(buf+30));
      this->rawPos[4] = *((int16_t *)(buf+32));
      this->rawPos[5] = *((int16_t *)(buf+34));
      this->rawQuat[4] = *((int16_t *)(buf+36));
      this->rawQuat[5] = *((int16_t *)(buf+38));
      this->rawQuat[6] = *((int16_t *)(buf+40));
      this->rawQuat[7] = *((int16_t *)(buf+42));
      this->rawButtons[1] = buf[44] & 0x7f;
      this->rawAnalog[3] = *((int16_t *)(buf+45));
      this->rawAnalog[4] = *((int16_t *)(buf+47));
      this->rawAnalog[5] = buf[49];

      boost::mutex::scoped_lock lock(this->mutex);
      // Put the raw position and orientation into Gazebo coordinate frame
      for (int i = 0; i < 2; i++)
      {
        this->pos[i].x = -this->rawPos[3*i+1] * 0.001;
        this->pos[i].y = -this->rawPos[3*i+0] * 0.001;
        this->pos[i].z = -this->rawPos[3*i+2] * 0.001;

        this->quat[i].w = this->rawQuat[i*4+0] / 32768.0;
        this->quat[i].x = -this->rawQuat[i*4+2] / 32768.0;
        this->quat[i].y = -this->rawQuat[i*4+1] / 32768.0;
        this->quat[i].z = -this->rawQuat[i*4+3] / 32768.0;
      }


      // Apply filters
      for (int i = 0; i < 2; i++)
      {
        this->quat[i] = this->filterQuat[i].Process(this->quat[i]);
        this->pos[i] = this->filterPos[i].Process(this->pos[i]);
      }

      this->analog[0] = this->rawAnalog[0] / 32768.0;
      this->analog[1] = this->rawAnalog[1] / 32768.0;
      this->analog[2] = this->rawAnalog[2] / 255.0;
      this->analog[3] = this->rawAnalog[3] / 32768.0;
      this->analog[4] = this->rawAnalog[4] / 32768.0;
      this->analog[5] = this->rawAnalog[5] / 255.0;

      for (int i = 0; i < 2; ++i)
      {
        this->buttons[i*7  ] = (this->rawButtons[i] & 0x01) ? 1 : 0;
        this->buttons[i*7+1] = (this->rawButtons[i] & 0x04) ? 1 : 0;
        this->buttons[i*7+2] = (this->rawButtons[i] & 0x08) ? 1 : 0;
        this->buttons[i*7+3] = (this->rawButtons[i] & 0x02) ? 1 : 0;
        this->buttons[i*7+4] = (this->rawButtons[i] & 0x10) ? 1 : 0;
        this->buttons[i*7+5] = (this->rawButtons[i] & 0x20) ? 1 : 0;
        this->buttons[i*7+6] = (this->rawButtons[i] & 0x40) ? 1 : 0;
      }

      return true;
    }
    else
    {
      common::Time::NSleep(250000);
      // gzerr << "Read failed\n";
    }
    /*else
    {
      common::Time toSleep =
        this->lastCycleStart +
        common::Time(this->periodEstimate.GetValue() * 0.95);

      common::Time sleepDuration = toSleep - common::Time::GetWallTime();

      if(sleepDuration > 0)
        common::Time::Sleep(sleepDuration);
      else
        common::Time::NSleep(250000);
    }*/
  //}

  return false;
}
