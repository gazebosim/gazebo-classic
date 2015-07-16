#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class RegionEventBoxPlugin : public ModelPlugin
  {
  	public: RegionEventBoxPlugin()
  		: ModelPlugin()
  	{
  	  this->receiveMutex = new boost::mutex();
  	  this->staleSizeAndPose = true;
  	}

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      std::cout << "RegionEventBoxPlugin::Load(): model=" << _parent->GetName() << std::endl << std::flush;

      this->model = _parent;
    	this->sdf = _sdf;

    	this->modelName = _parent->GetName();
    	this->world = _parent->GetWorld();

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->modelSub = this->node->Subscribe("~/model/info", &RegionEventBoxPlugin::OnModelMsg, this);
      this->poseSub = this->node->Subscribe("~/pose/info", &RegionEventBoxPlugin::OnPoseMsg, this);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RegionEventBoxPlugin::OnUpdate, this, _1));

    }

  	public: void OnModelMsg(ConstModelPtr & _msg)
  	{
  		std::string modelMsgName = _msg->name();

  	  boost::mutex::scoped_lock lock(*this->receiveMutex);
			std::cerr << "RegionEventBoxPlugin::OnModelMsg(): name=" << modelMsgName << std::endl << std::flush;

			if (_msg->has_name() && _msg->name() == this->modelName)
			{
				this->staleSizeAndPose = true;
			}

  	}

  	public:void OnPoseMsg(ConstPosesStampedPtr &_msg)
  	{
  	  boost::mutex::scoped_lock lock(*this->receiveMutex);
			std::cout << "RegionEventBoxPlugin::OnPoseMsg()..." << std::endl << std::flush;

			for (int i = 0; i < _msg->pose_size(); i++)
			{
				const gazebo::msgs::Pose &p = _msg->pose(i);

				if (p.name() == this->modelName)
				{
					this->staleSizeAndPose = true;
				}
			}

  	}

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
			for (unsigned int i = 0; i < this->world->GetModelCount(); i++)
			{
				physics::ModelPtr m = this->world->GetModel(i);
				std::string name = m->GetName();

				if (name == "ground_plane")
				{
					continue;
				}

				if (name == this->modelName)
				{
					if (this->staleSizeAndPose)
					{
						std::cout << "RegionEventPlugin::OnUpdate(): updating size and pose for model \"" << name << "\"" << std::endl << std::flush;

						if (!this->UpdateSizeAndPose(m->GetSDF(), m->GetWorldPose()))
						{
							std::cerr << "RegionEventPlugin::OnUpdate(): failed to update size and pose for model \"" << name << "\"" << std::endl << std::flush;
							return;
						}
						this->staleSizeAndPose = false;
					}
					continue;
				}

				std::cout << "RegionEventPlugin::OnUpdate(): another model \"" << m->GetName() << "\"" << std::endl << std::flush;
				std::cout << "   pose=" << m->GetWorldPose().pos << std::endl << std::flush;
				std::cout << "   bbox=" << this->box << std::endl << std::flush;

				if (this->box.Contains(m->GetWorldPose().pos))
				{
					std::cout << "   INSIDE!!!!!" << std::endl << std::flush;
				}
				else
				{
					std::cout << "   outside" << std::endl << std::flush;
				}


			}	//	if (this->modelName == m->GetName())

    }

    private: bool UpdateSizeAndPose(sdf::ElementPtr _sdf, const math::Pose& _pose)
    {
			sdf::ElementPtr linkEl = _sdf->GetElement("link");
			if (linkEl)
			{
				sdf::ElementPtr visualEl = linkEl->GetElement("visual");
				if (visualEl)
				{
					sdf::ElementPtr geometryEl = visualEl->GetElement("geometry");
					if (geometryEl)
					{
						sdf::ElementPtr boxEl = geometryEl->GetElement("box");
						if (boxEl)
						{
							if (boxEl->HasElement("size"))
							{
								std::string ssize = boxEl->Get<std::string>("size");

								this->size = this->ParseVector3(ssize);
					    	this->pose = _pose;

					    	math::Vector3 vmin(this->pose.pos.x - (this->size.x / 2), this->pose.pos.y - (this->size.y / 2), 0.0 /*this->pose.pos.z*/);
					    	math::Vector3 vmax(this->pose.pos.x + (this->size.x / 2), this->pose.pos.y + (this->size.y / 2), this->pose.pos.z + this->size.z);

					    	this->box = math::Box(vmin, vmax);

								return true;
							}
						}
					}
				}
			}
			return false;
    }

    private: math::Vector3 ParseVector3(const std::string &_str, double _scale = 1.0)
    {
      std::vector<std::string> pieces;
      std::vector<double> vals;

      boost::split(pieces, _str, boost::is_any_of(" "));
      for (unsigned int i = 0; i < pieces.size(); ++i)
      {
        if (pieces[i] != "")
        {
          try
          {
            vals.push_back(_scale * boost::lexical_cast<double>(pieces[i].c_str()));
          }
          catch(boost::bad_lexical_cast &)
          {
            sdferr << "xml key [" << _str
              << "][" << i << "] value [" << pieces[i]
              << "] is not a valid double from a 3-tuple\n";
            return math::Vector3(0, 0, 0);
          }
        }
      }

      if (vals.size() == 3)
      {
        return math::Vector3(vals[0], vals[1], vals[2]);
      }
      else
      {
        return math::Vector3(0, 0, 0);
      }

    }

    private: physics::WorldPtr world;

    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;

    private: sdf::ElementPtr sdf;

    private: std::string modelName;

    public: transport::NodePtr node;

    public: boost::mutex *receiveMutex;

    private: math::Vector3 size;
    private: math::Pose pose;
    private: math::Box box;

    private: transport::SubscriberPtr modelSub;
    private: transport::SubscriberPtr poseSub;

    private: bool staleSizeAndPose;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RegionEventBoxPlugin)

}
