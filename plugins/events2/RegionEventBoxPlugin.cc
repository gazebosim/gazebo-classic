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
  	}

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
    	this->model = _parent;
    	this->sdf = _sdf;
    	this->world = _parent->GetWorld();
    	this->modelName = _parent->GetName();

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      transport::SubscriberPtr sp = this->node->Subscribe("~/model/info", &RegionEventBoxPlugin::OnModelMsg, this);
      std::cerr << "RegionEventBoxPlugin::Load(): node=" << this->node->GetId( )<< " (" << this->node << ") subscription=" << sp << std::endl << std::flush;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RegionEventBoxPlugin::OnUpdate, this, _1));
    }

  	public: void OnModelMsg(ConstModelPtr & _msg)
  	{
  		(void)_msg;

  	  boost::mutex::scoped_lock lock(*this->receiveMutex);
			std::cerr << "RegionEventBoxPlugin::OnModelMsg()..." << std::endl << std::flush;
  	}

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
    		for (unsigned int i = 0; i < this->world->GetModelCount(); i++) {

    			physics::ModelPtr m = this->world->GetModel(i);

    			if (this->modelName == m->GetName()) {

						if (!this->UpdateSizeAndPose(m->GetSDF(), m->GetWorldPose()))
						{
							std::cerr << "Failed to update size and position for model " << m->GetName() << std::endl << std::flush;
							return;
						}
						std::cout << "size=" << this->size << std::endl << std::flush;
						std::cout << "pose=" << this->pose << std::endl << std::flush;
    			}

    		}

    }

    private: bool UpdateSizeAndPose(sdf::ElementPtr _sdf, const math::Pose& _pose)
    {
    	bool result = false;

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
								result = true;
							}
						}
					}
				}
			}
			return result;
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

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RegionEventBoxPlugin)

}
