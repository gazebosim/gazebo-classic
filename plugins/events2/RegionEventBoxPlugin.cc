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

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
    		for (unsigned int i = 0; i < this->world->GetModelCount(); i++) {

    			physics::ModelPtr m = this->world->GetModel(i);

    			if (this->modelName == m->GetName()) {

//    				math::Vector3 pos = m->GetWorldPose().pos;
//    				math::Box bbox = m->GetBoundingBox();
//    				math::Pose pose = m->GetWorldPose();

						sdf::ElementPtr modelSdf = m->GetSDF();

						sdf::ElementPtr linkEl = modelSdf->GetElement("link");
						if (linkEl) {
							sdf::ElementPtr visualEl = linkEl->GetElement("visual");
							if (visualEl) {
								sdf::ElementPtr geometryEl = visualEl->GetElement("geometry");
								if (geometryEl) {
									sdf::ElementPtr boxEl = geometryEl->GetElement("box");
									if (boxEl) {
										if (boxEl->HasElement("size")) {
											std::string size = boxEl->Get<std::string>("size");
//											std::cerr << "  size=" << size << std::endl << std::flush;
										}
									}
								}
							}
						}

    			}

    		}

    }

    private: physics::WorldPtr world;

    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;

    private: sdf::ElementPtr sdf;

    private: std::string modelName;

    public: transport::NodePtr node;

    public: boost::mutex *receiveMutex;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RegionEventBoxPlugin)

}
