#ifndef _GAZEBO_RAZER_HYDRA_HH_
#define _GAZEBO_RAZER_HYDRA_HH_

#include <stdint.h>
//#include <fcntl.h>
//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <sys/ioctl.h>
//#include <linux/hidraw.h>
//#include <libusb-1.0/libusb.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  /// \brief Filter base class
  template <class T>
  class Filter
  {
    public: virtual void SetValue(const T &_val) { y0 = _val; }
    public: virtual void SetFc(double _fc, double _fs) = 0;
    public: inline virtual const T& GetValue() { return y0; }

    protected: T y0;
  };

  /// \brief A one-pole DSP filter.
  /// \sa http://www.earlevel.com/main/2012/12/15/a-one-pole-filter/
  template <class T>
  class OnePole : public Filter<T>
  {
    public: OnePole() : a0(0), b1(0) {}

    public: OnePole(double _fc, double _fs)
            : a0(0), b1(0)
            {
              this->SetFc(_fc, _fs);
            }

    public: virtual void SetFc(double _fc, double _fs)
            {
              b1 = exp(-2.0 * M_PI * _fc / _fs);
              a0 = 1.0 - b1;
            }

    public: inline const T& Process(const T &_x)
            {
              this->y0 = a0 * _x + b1 * this->y0;
              return this->y0;
            }

    protected: double a0;
    protected: double b1;
  };

  /// \brief One-pole quaternion filter.
  class OnePoleQuaternion : public OnePole<math::Quaternion>
  {
    public: OnePoleQuaternion()
            {
              this->SetValue(math::Quaternion(1,0,0,0));
            }

    public: OnePoleQuaternion(double _fc, double _fs)
            : OnePole<math::Quaternion>(_fc, _fs)
            {
              this->SetValue(math::Quaternion(1,0,0,0));
            }

    public: inline const math::Quaternion& Process(const math::Quaternion &_x)
            {
              y0 = math::Quaternion::Slerp(a0, y0, _x);
              return y0;
            }
  };

  /// \brief One-pole vector3 filter.
  class OnePoleVector3 : public OnePole<math::Vector3>
  {
    public: OnePoleVector3()
            {
              this->SetValue(math::Vector3(0,0,0));
            }

    public: OnePoleVector3(double _fc, double _fs)
            : OnePole<math::Vector3>(_fc, _fs)
            {
              this->SetValue(math::Vector3(0,0,0));
            }
  };

  /// \brief Bi-quad filter base class.
  /// \sa http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/
  template <class T>
  class BiQuad : public Filter<T>
  {
    public: BiQuad()
            : a0(0), a1(0), a2(0), b0(0), b1(0), b2(0) { }

    public: BiQuad(double _fc, double _fs)
            : a0(0), a1(0), a2(0), b0(0), b1(0), b2(0)
            {
              this->SetFc(_fc, _fs);
            }

    public: inline void SetFc(double _fc, double _fs)
            {
              this->SetFc(_fc, _fs, 0.5);
            }

    public: inline void SetFc(double _fc, double _fs, double _q)
            {
              double k = tan(M_PI * _fc / _fs);
              double kQuadDenom = k * k + k / _q + 1.0;
              this->a0 = k * k/ kQuadDenom;
              this->a1 = 2 * this->a0;
              this->a2 = this->a0;
              this->b0 = 1.0;
              this->b1 = 2 * (k * k - 1.0) / kQuadDenom;
              this->b2 = (k * k - k / _q + 1.0) / kQuadDenom;
            }

    public: virtual void SetValue(const T &_val)
            {
              this->y0 = this->y1 = this->y2 = this->x1 = this->x2 = _val;
            }

    public: inline virtual const T& process(const T &_x)
            {
              this->y0 = this->a0 * _x +
                         this->a1 * this->x1 +
                         this->a2 * this->x2 -
                         this->b1 * this->y1 -
                         this->b2 * this->y2;

              this->x2 = this->x1;
              this->x1 = _x;
              this->y2 = this->y1;
              this->y1 = this->y0;
              return this->y0;
            }

    protected: double a0, a1, a2, b0, b1, b2;
    protected: T x1, x2, y1, y2;
  };

  /// \brief BiQuad vector3 filter
  class BiQuadVector3 : public BiQuad<math::Vector3>
  {
    public: BiQuadVector3()
            {
              this->SetValue(math::Vector3(0,0,0));
            }

    public: BiQuadVector3(double _fc, double _fs)
            : BiQuad<math::Vector3>(_fc, _fs)
            {
              this->SetValue(math::Vector3(0,0,0));
            }
  };

  class RazerHydra : public WorldPlugin
  {
    /// \brief Constructor.
    public: RazerHydra();

    /// \brief Destructor.
    public: virtual ~RazerHydra();

    /// \brief Load and initialize the hydra.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Poll the hydra for input.
    private: bool Poll(const common::Time &_timeToWait,
                      float lowPassCornerHz = 5.0);

    private: void Run();

    /// \brief Update the hydra.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief
    private: int16_t rawPos[6];

    /// \brief
    private: int16_t rawQuat[8];

    /// \brief
    private: uint8_t rawButtons[2];

    /// \brief
    private: double rawAnalog[6];

    /// \brief
    private: int hidrawFd;

    /// \brief
    private: math::Vector3 pos[2];

    /// \brief
    private: math::Quaternion quat[2];

    /// \brief
    private: OnePoleVector3 filterPos[2];

    /// \brief
    private: OnePoleQuaternion filterQuat[2];

    /// \brief
    private: float analog[6];

    /// \brief Buttons that have been pressed.
    private: uint8_t buttons[14];

    /// \brief Estimate of the update period.
    private: OnePole<float> periodEstimate;

    /// \brief Time of the last poll cycle.
    private: common::Time lastCycleStart;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr rightModel;
    private: physics::ModelPtr leftModel;

    private: math::Pose basePoseRight, resetPoseRight;
    private: math::Pose basePoseLeft, resetPoseLeft;

    private: boost::mutex mutex;
    private: boost::thread *pollThread;
    private: bool stop;

    private: uint8_t prevState;

    private: transport::NodePtr node;
    private: transport::PublisherPtr pub;
  };
}
#endif