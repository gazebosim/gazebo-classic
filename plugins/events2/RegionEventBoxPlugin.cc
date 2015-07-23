#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <EventSource.hh>
#include <stdio.h>
#include <map>

namespace gazebo
{
  class RegionEventBoxPlugin : public ModelPlugin
  {
  	public: RegionEventBoxPlugin()
  		: ModelPlugin(), eventPub(0)
  	{
  	  this->receiveMutex = new boost::mutex();
  	  this->hasStaleSizeAndPose = true;
  	}

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      std::cout << "RegionEventBoxPlugin::Load(): model=\"" << _parent->GetName() << "\"" << std::endl << std::flush;

    	this->modelName = _parent->GetName();
    	this->world = _parent->GetWorld();

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->modelSub = this->node->Subscribe("~/model/info", &RegionEventBoxPlugin::OnModelMsg, this);
      this->poseSub = this->node->Subscribe("~/pose/info", &RegionEventBoxPlugin::OnPoseMsg, this);

      if (_sdf->HasElement("event")) {

      	sdf::ElementPtr event = _sdf->GetElement("event");
      	std::string eventType = event->Get<std::string>("type");

      	if (eventType == "inclusion") {
          this->eventPub = this->node->Advertise<gazebo::msgs::SimEvent>("/gazebo/sim_events");
          this->eventSource = gazebo::EventSourcePtr(new EventSource(eventPub, eventType, this->world));
      		this->eventSource->Load(event);
      	}

      }
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RegionEventBoxPlugin::OnUpdate, this, _1));
    }

  	public: void OnModelMsg(ConstModelPtr & _msg)
  	{
  	  boost::mutex::scoped_lock lock(*this->receiveMutex);

			if (_msg->has_name() && _msg->name() == this->modelName)
			{
				this->hasStaleSizeAndPose = true;
			}

  	}

  	public:void OnPoseMsg(ConstPosesStampedPtr &_msg)
  	{
  	  boost::mutex::scoped_lock lock(*this->receiveMutex);

			for (int i = 0; i < _msg->pose_size(); i++)
			{
				const gazebo::msgs::Pose &p = _msg->pose(i);

				if (p.name() == this->modelName)
				{
					this->hasStaleSizeAndPose = true;
				}
			}

  	}

    public: void OnUpdate(const common::UpdateInfo & _info)
    {
    	//std::cout << "RegionEventBoxPlugin::OnUpdate()..." << std::endl << std::flush;

			for (unsigned int i = 0; i < this->world->GetModelCount(); i++)
			{
				physics::ModelPtr model = this->world->GetModel(i);
				std::string name = model->GetName();

				if (name == "ground_plane")
					continue;

				if (name == this->modelName)
				{
					if (this->hasStaleSizeAndPose)
					{
						if (!this->UpdateSizeAndPose(model->GetSDF(), model->GetWorldPose()))
						{
							std::cerr << "RegionEventPlugin::OnUpdate(): failed to update size and pose for model \"" << name << "\"" << std::endl << std::flush;
							return;
						}
						this->hasStaleSizeAndPose = false;
					}
					continue;
				}

	    	std::map<std::string, common::Time>::iterator it = this->insiders.find(model->GetName());

				if (this->box.Contains(model->GetWorldPose().pos))
				{
					if (it == this->insiders.end()) {
						this->insiders[model->GetName()] = _info.realTime;
						if (this->eventPub) {
							this->SendEnteringRegionEvent(model);
						}
					}
				}
				else
				{
					if (it != this->insiders.end()) {
						if (this->eventPub) {
							this->SendExitingRegionEvent(model);
						}
						this->insiders.erase(model->GetName());
					}
				}

			}	//	if (this->modelName == m->GetName())

    }

    private: bool UpdateSizeAndPose(sdf::ElementPtr _sdf, const math::Pose& _pose)
    {
			std::cout << "RegionEventPlugin::UpdateSizeAndPose(): model=\"" << this->modelName << "\"" << std::endl << std::flush;

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

    private: void SendEnteringRegionEvent(physics::ModelPtr model)
    {
			std::cout << "RegionEventBoxPlugin::SendEnteringRegionEvent(): model=\"" << model->GetName() << "\""
					" region=\"" << this->modelName << "\"" << std::endl << std::flush;

			std::string json = "{";
	    json += "\"state\":\"inside\",";
	    json += "\"region\":\"" + this->modelName + "\", ";
	    json += "\"model\":\"" + model->GetName() + "\"";
	    json += "}";

	    eventSource->Emit(json);
    }

    private: void SendExitingRegionEvent(physics::ModelPtr model)
    {
			std::cout << "RegionEventBoxPlugin::SendExitingRegionEvent(): model=\"" << model->GetName() << "\""
					" region=\"" << this->modelName << "\"" << std::endl << std::flush;

			std::string json = "{";
	    json += "\"state\":\"outside\",";
	    json += "\"region\":\"" + this->modelName + "\", ";
	    json += "\"model\":\"" + model->GetName() + "\"";
	    json += "}";

	    eventSource->Emit(json);
    }

    private: physics::WorldPtr world;

    private: event::ConnectionPtr updateConnection;

    private: std::string modelName;

    public: transport::NodePtr node;

    public: boost::mutex *receiveMutex;

    private: math::Vector3 size;
    private: math::Pose pose;
    private: math::Box box;

    private: transport::SubscriberPtr modelSub;
    private: transport::SubscriberPtr poseSub;

    private: bool hasStaleSizeAndPose;
    private: std::map<std::string, common::Time> insiders;

    private: transport::PublisherPtr eventPub;
    private: gazebo::EventSourcePtr eventSource;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RegionEventBoxPlugin)

}
