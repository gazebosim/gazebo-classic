/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/VisualPrivate.hh"
#include "gazebo/rendering/MarkerVisual.hh"

using namespace gazebo;
using namespace rendering;

/// \brief Private data for the marker Visual class.
class gazebo::rendering::MarkerVisualPrivate : public VisualPrivate
{
  /// \brief Renders line segments
  public: std::unique_ptr<DynamicLines> dynamicRenderable;

  /// \brief Renders text.
  public: std::unique_ptr<MovableText> text;

  /// \brief Mutex to protect processing messages.
  public: std::mutex mutex;

  /// \brief The last marker message received.
  public: ignition::msgs::Marker msg;

  /// \brief Lifetime of the marker
  public: common::Time lifetime;

  /// \brief True when the marker has already been loaded.
  public: bool loaded = false;
};

/////////////////////////////////////////////////
MarkerVisual::MarkerVisual(const std::string &_name, VisualPtr _vis)
: Visual(*new MarkerVisualPrivate, _name, _vis, false)
{
  this->dPtr = reinterpret_cast<MarkerVisualPrivate *>(this->dataPtr);
}

/////////////////////////////////////////////////
MarkerVisual::~MarkerVisual()
{
  this->Fini();
  this->dPtr = nullptr;
}

/////////////////////////////////////////////////
void MarkerVisual::Load(const ignition::msgs::Marker &_msg)
{
  std::lock_guard<std::mutex> lock(this->dPtr->mutex);

  if (!this->dPtr->loaded)
    Visual::Load();

  if (_msg.action() == ignition::msgs::Marker::ADD_MODIFY)
  {
    this->AddModify(_msg);
  }

  this->dPtr->loaded = true;
}

/////////////////////////////////////////////////
void MarkerVisual::AddModify(const ignition::msgs::Marker &_msg)
{
  bool dynamicRenderableCalled = false;

  // Set the type of visual
  if (this->dPtr->msg.type() != _msg.type() && _msg.has_type())
  {
    this->dPtr->msg.set_type(_msg.type());
    switch (_msg.type())
    {
      case ignition::msgs::Marker::BOX:
        this->DetachObjects();
        this->AttachMesh("unit_box");
        break;
      case ignition::msgs::Marker::CYLINDER:
        this->DetachObjects();
        this->AttachMesh("unit_cylinder");
        break;
      case ignition::msgs::Marker::LINE_STRIP:
      case ignition::msgs::Marker::LINE_LIST:
      case ignition::msgs::Marker::POINTS:
      case ignition::msgs::Marker::TRIANGLE_FAN:
      case ignition::msgs::Marker::TRIANGLE_LIST:
      case ignition::msgs::Marker::TRIANGLE_STRIP:
        this->DynamicRenderable(_msg);
        dynamicRenderableCalled = true;
        break;
      case ignition::msgs::Marker::SPHERE:
        this->DetachObjects();
        this->AttachMesh("unit_sphere");
        break;
      case ignition::msgs::Marker::TEXT:
        this->Text(_msg);
        break;
      default:
        gzerr << "Unable to create marker of type[" << _msg.type() << "]\n";
        break;
    };
  }

  // Set the marker's material
  if (_msg.has_material())
  {
    this->ProcessMaterialMsg(_msg.material());
  }

  // Scale the visual
  if (_msg.has_scale())
  {
    this->SetScale(ignition::math::Vector3d(
        _msg.scale().x(), _msg.scale().y(), _msg.scale().z()));
  }

  // Set the visual's pose
  if (_msg.has_pose())
    this->SetPose(ignition::msgs::Convert(_msg.pose()));

  // Set the marker's end time
  if (_msg.has_lifetime() &&
      (_msg.lifetime().sec() > 0 ||
      (_msg.lifetime().sec() == 0 && _msg.lifetime().nsec() > 0)))
  {
    this->dPtr->lifetime = this->GetScene()->SimTime() +
      gazebo::common::Time(_msg.lifetime().sec(), _msg.lifetime().nsec());
  }

  // Attach marker to a parent visual, if specified in the message.
  if (_msg.has_parent())
  {
    // Detach from existing parent. Only detach if a parent exists
    // and is not the root node when the new parent name is not empty.
    if (this->GetParent())
      this->GetParent()->DetachVisual(shared_from_this());

    // Get the new parent
    VisualPtr parent = this->GetScene()->GetVisual(_msg.parent());

    // Attach to the new parent, if the parent is valid
    if (parent)
      parent->AttachVisual(shared_from_this());
    else if (_msg.parent().empty())
      this->GetScene()->WorldVisual()->AttachVisual(shared_from_this());
    else if (!_msg.parent().empty())
      gzerr << "No visual with the name[" << _msg.parent() << "]\n";
  }

  if (_msg.has_layer())
  {
    rendering::Events::newLayer(_msg.layer());
    this->SetLayer(_msg.layer());
  }

  if (!dynamicRenderableCalled &&
      _msg.point_size() && this->dPtr->dynamicRenderable)
  {
    this->DynamicRenderable(_msg);
  }

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
common::Time MarkerVisual::Lifetime() const
{
  return this->dPtr->lifetime;
}

/////////////////////////////////////////////////
void MarkerVisual::DynamicRenderable(const ignition::msgs::Marker &_msg)
{
  if (!this->dPtr->dynamicRenderable)
  {
    switch (_msg.type())
    {
      case ignition::msgs::Marker::LINE_STRIP:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_LINE_STRIP));
        break;
      case ignition::msgs::Marker::LINE_LIST:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_LINE_LIST));
        break;
      case ignition::msgs::Marker::POINTS:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_POINT_LIST));
        break;
      case ignition::msgs::Marker::TRIANGLE_FAN:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
        break;
      case ignition::msgs::Marker::TRIANGLE_LIST:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_LIST));
        break;
      case ignition::msgs::Marker::TRIANGLE_STRIP:
        this->dPtr->dynamicRenderable.reset(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_STRIP));
        break;
      default:
        gzerr << "Unable to create dynamic renderable of type[" <<
          _msg.type() << "]\n";
        break;
    };
  }
  else
  {
    // Change render operation, if present
    if (_msg.has_type())
    {
      switch (_msg.type())
      {
        case ignition::msgs::Marker::LINE_STRIP:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_LINE_STRIP)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_LINE_STRIP);
          }
          break;
        case ignition::msgs::Marker::LINE_LIST:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_LINE_LIST)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_LINE_LIST);
          }
          break;
        case ignition::msgs::Marker::POINTS:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_POINT_LIST)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_POINT_LIST);
          }
          break;
        case ignition::msgs::Marker::TRIANGLE_FAN:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_TRIANGLE_FAN)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_TRIANGLE_FAN);
          }
          break;
        case ignition::msgs::Marker::TRIANGLE_LIST:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_TRIANGLE_LIST)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_TRIANGLE_LIST);
          }
          break;
        case ignition::msgs::Marker::TRIANGLE_STRIP:
          if (this->dPtr->dynamicRenderable->GetOperationType() !=
              rendering::RENDERING_TRIANGLE_STRIP)
          {
            this->dPtr->dynamicRenderable->SetOperationType(
                rendering::RENDERING_TRIANGLE_STRIP);
          }
          break;
        default:
          break;
      };
    }

    // We make the assumption that the presence of points means the existing
    // points should be removed.
    if (_msg.point_size() > 0)
      this->dPtr->dynamicRenderable->Clear();
  }

  for (int i = 0; i < _msg.point_size(); ++i)
  {
    this->dPtr->dynamicRenderable->AddPoint(
        ignition::math::Vector3d(_msg.point(i).x(),
                                 _msg.point(i).y(),
                                 _msg.point(i).z()));
  }
}

/////////////////////////////////////////////////
void MarkerVisual::Text(const ignition::msgs::Marker &_msg)
{
  if (!this->dPtr->text)
  {
    this->dPtr->text.reset(new MovableText());
    this->dPtr->text->Load(this->Name() + "__TEXT__", _msg.text());
    this->GetSceneNode()->attachObject(this->dPtr->text.get());
  }
  else
  {
    this->dPtr->text->SetText(_msg.text());
    this->dPtr->text->Update();
  }
}

/////////////////////////////////////////////////
void MarkerVisual::Fini()
{
  if (this->dPtr->dynamicRenderable)
    this->DeleteDynamicLine(this->dPtr->dynamicRenderable.release());

  if (this->dPtr->text && this->dPtr->text->getParentNode())
  {
    this->dPtr->text->detachFromParent();
    this->GetSceneNode()->detachObject(this->dPtr->text.get());
    this->dPtr->text.reset();
  }
  Visual::Fini();
}

/////////////////////////////////////////////////
void MarkerVisual::FillMsg(ignition::msgs::Marker &_msg)
{
  _msg.mutable_lifetime()->set_sec(this->dPtr->lifetime.sec);
  _msg.mutable_lifetime()->set_nsec(this->dPtr->lifetime.nsec);
  ignition::msgs::Set(_msg.mutable_pose(), this->Pose());

  if (this->dPtr->text)
    _msg.set_text(this->dPtr->text->Text());

  if (this->GetParent())
    _msg.set_parent(this->GetParent()->Name());

  // Set the scale
  ignition::msgs::Set(_msg.mutable_scale(), this->dataPtr->scale);

  // Add points, if present
  for (unsigned int count = 0; this->dPtr->dynamicRenderable &&
      count < this->dPtr->dynamicRenderable->GetPointCount(); ++count)
  {
    ignition::msgs::Set(_msg.add_point(),
        this->dPtr->dynamicRenderable->Point(count));
  }

  this->FillMaterialMsg(*(_msg.mutable_material()));

  _msg.set_layer(this->dataPtr->layer);
  _msg.set_type(this->dPtr->msg.type());
}
