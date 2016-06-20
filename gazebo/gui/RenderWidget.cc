/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/bind.hpp>
#include <iomanip>

#include "gazebo/common/CommonIface.hh"

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/TopToolbar.hh"
#include "gazebo/gui/RenderWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RenderWidget::RenderWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("renderWidget");

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->mainFrame = new QFrame;
  this->mainFrame->setFrameShape(QFrame::NoFrame);
  this->mainFrame->setContentsMargins(0, 0, 0, 0);
  this->mainFrame->show();
  mainLayout->addWidget(this->mainFrame);
  this->setLayout(mainLayout);

  this->glWidget = new GLWidget(this->mainFrame);

  QFrame *render3DFrame = new QFrame;
  render3DFrame->setObjectName("myrenderframe");
  render3DFrame->setFrameShape(QFrame::NoFrame);
  render3DFrame->setContentsMargins(0, 0, 0, 0);

  QVBoxLayout *render3DLayout = new QVBoxLayout;
  render3DLayout->setSpacing(0);
  render3DLayout->setContentsMargins(0, 0, 0, 0);

  //QVBoxLayout *frameLayout = new QVBoxLayout;

  /*this->toolFrame = new QFrame;
  this->toolFrame->setObjectName("toolFrame");
  this->toolFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  */

  // GLWigdet
  this->glWidget = new GLWidget(this->mainFrame);

  this->msgOverlayLabel = new QLabel(this->glWidget);
  this->msgOverlayLabel->setStyleSheet(
      "QLabel { background-color : white; color : gray; }");
  this->msgOverlayLabel->setVisible(false);
/*
  QHBoxLayout *bottomPanelLayout = new QHBoxLayout;

  this->timePanel = new TimePanel(this);

  this->bottomFrame = new QFrame;
  this->bottomFrame->setObjectName("renderBottomFrame");
  this->bottomFrame->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Minimum);

  bottomPanelLayout->addWidget(this->timePanel, 0);
  bottomPanelLayout->setSpacing(0);
  bottomPanelLayout->setContentsMargins(0, 0, 0, 0);
  this->bottomFrame->setLayout(bottomPanelLayout);
  */

  /*this->splitter = new QSplitter(this);
  this->splitter->addWidget(render3DFrame);
  QList<int> sizes;
  sizes.push_back(300);
  this->splitter->setSizes(sizes);
  this->splitter->setStretchFactor(0, 1);
  this->splitter->setOrientation(Qt::Vertical);

  frameLayout->addWidget(this->splitter);
  frameLayout->addWidget(this->bottomFrame);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  */
  //this->mainFrame->setLayout(frameLayout);
  //this->mainFrame->layout()->setContentsMargins(0, 0, 0, 0);

/*
  this->connections.push_back(
      gui::Events::ConnectFollow(
        boost::bind(&RenderWidget::OnFollow, this, _1)));

  // Load all GUI Plugins
  std::string filenames = getINIProperty<std::string>(
      "overlay_plugins.filenames", "");

  // Split the colon separated libraries
  auto pluginFilenames = common::split(filenames, ":");

  // Load each plugin
  for (std::vector<std::string>::iterator iter = pluginFilenames.begin();
       iter != pluginFilenames.end(); ++iter)
  {
    // Make sure the string is not empty
    if (!(*iter).empty())
    {
      // Try to create the plugin
      gazebo::GUIPluginPtr plugin = gazebo::GUIPlugin::Create(*iter, *iter);

      if (!plugin)
      {
        gzerr << "Unable to create gui overlay plugin with filename["
          << *iter << "]\n";
      }
      else
      {
        gzlog << "Loaded GUI plugin[" << *iter << "]\n";

        // Set the plugin's parent and store the plugin
        plugin->setParent(this->glWidget);
        this->plugins.push_back(plugin);
      }
    }
  }
*/
}

/////////////////////////////////////////////////
RenderWidget::~RenderWidget()
{
  delete this->glWidget;
  this->glWidget = NULL;

  delete this->topToolbar;
  this->topToolbar = NULL;

  // we created the scene here we are responsible for removing it.
  rendering::remove_scene(gui::get_world());
}

/////////////////////////////////////////////////
void RenderWidget::InsertWidget(unsigned int _index, QWidget *_widget)
{
  if (static_cast<int>(_index) <= this->splitter->count())
  {
    // set equal size for now. There should always be at least one widget
    // (render3DFrame) in the splitter.
    int childCount = this->splitter->count();
    GZ_ASSERT(childCount > 0,
        "RenderWidget splitter has no child widget");

    QSize widgetSize = this->size();
    int newSize = widgetSize.height() / (this->splitter->count()+1);
    QList<int> newSizes;
    for (int i = 0; i < childCount+1; ++i)
      newSizes.append(newSize);

    this->splitter->insertWidget(_index, _widget);
    this->splitter->setSizes(newSizes);
    this->splitter->setStretchFactor(_index, 1);
  }
  else
    gzerr << "Unable to add widget, index out of range " << std::endl;
}

/////////////////////////////////////////////////
void RenderWidget::ShowTimePanel(bool _show)
{
  if (_show)
    this->bottomFrame->show();
  else
    this->bottomFrame->hide();
}

/////////////////////////////////////////////////
TimePanel *RenderWidget::GetTimePanel() const
{
  return this->timePanel;
}

/////////////////////////////////////////////////
void RenderWidget::RemoveScene(const std::string &_name)
{
  rendering::remove_scene(_name);
}

/////////////////////////////////////////////////
void RenderWidget::CreateScene(const std::string &_name)
{
  rendering::create_scene(_name, true);
}

/////////////////////////////////////////////////
void RenderWidget::DisplayOverlayMsg(const std::string &_msg, int _duration)
{
  std::string msg = this->baseOverlayMsg.empty() ? _msg
      : this->baseOverlayMsg + "\n" + _msg;
  this->msgOverlayLabel->setText(tr(msg.c_str()));
  if (msg.empty())
  {
    this->msgOverlayLabel->setVisible(false);
    return;
  }
  this->msgOverlayLabel->resize(
      this->msgOverlayLabel->fontMetrics().width(tr(msg.c_str())),
      this->msgOverlayLabel->fontMetrics().height());
  this->msgOverlayLabel->setVisible(true);

  if (_duration > 0)
    QTimer::singleShot(_duration, this, SLOT(OnClearOverlayMsg()));
}

/////////////////////////////////////////////////
void RenderWidget::SetOverlaysVisible(const bool _visible)
{
  for (auto const &plugin : this->plugins)
    plugin->setVisible(_visible);
}

/////////////////////////////////////////////////
std::string RenderWidget::GetOverlayMsg() const
{
  return this->msgOverlayLabel->text().toStdString();
}

/////////////////////////////////////////////////
void RenderWidget::ShowToolbar(const bool _show)
{
  return;
  if (this->topToolbar)
  {
    if (_show)
    {
      this->topToolbar->show();
    }
    else
    {
      this->topToolbar->hide();
    }
  }
}

/////////////////////////////////////////////////
TopToolbar *RenderWidget::GetToolbar() const
{
  return NULL;
  return this->topToolbar;
}

/////////////////////////////////////////////////
void RenderWidget::OnClearOverlayMsg()
{
  this->DisplayOverlayMsg("");
}

/////////////////////////////////////////////////
void RenderWidget::OnFollow(const std::string &_modelName)
{
  if (_modelName.empty())
  {
    g_translateAct->setEnabled(true);
    g_rotateAct->setEnabled(true);
  }
  else
  {
    g_translateAct->setEnabled(false);
    g_rotateAct->setEnabled(false);
  }
}

/////////////////////////////////////////////////
void RenderWidget::AddPlugin(GUIPluginPtr _plugin, sdf::ElementPtr _elem)
{
  // Set the plugin's parent and store the plugin
  _plugin->setParent(this->glWidget);
  this->plugins.push_back(_plugin);

  // Load the plugin.
  _plugin->Load(_elem);

  _plugin->show();
}
