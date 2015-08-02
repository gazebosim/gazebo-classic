/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GUI_EXAMPLE_SPAWN_WIDGET_HH_
#define _GUI_EXAMPLE_SPAWN_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

namespace gazebo
{

  namespace gui
  {
    class ImageFrame;
  }

  class GAZEBO_VISIBLE CameraLensControlExample : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent widget
    public: CameraLensControlExample();

    /// \brief Destructor
    public: virtual ~CameraLensControlExample();

    protected: virtual void LoadGUIComponents(QWidget *_parent);

    /// \brief Callbacks trigged when the buttons are pressed.
    protected slots: void OnButtonSpawn();
    protected slots: void OnButtonCalibrate();

    protected slots: void OnCbTypeChange();
    protected slots: void OnCbFunChange();  

    protected slots: void OnSbChange();

    protected slots: void OnValueChanged();

    protected: void OnSelect(ConstSelectionPtr &_msg);

    protected: void OnImageUpdate(ConstImageStampedPtr &_msg);

    protected: void OnCameraProjectionCmd(ConstCameraLensCmdPtr &_msg);

    /// \brief Counter used to create unique model names.
    private: unsigned int counter;

    // \brief Name of current element selected in gazebo.
    protected: std::string selectedElementName;

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    /// \brief Publisher of factory messages.
    private: transport::PublisherPtr factoryPub;

    /// \brief Publisher of camera control messages.
    private: transport::PublisherPtr cameraControlPub;

    /// \brief Subscriber to selection messages.
    private: transport::SubscriberPtr selectionSub;

    /// \brief Subscriber to camera image messages.
    private: transport::SubscriberPtr imageSub;

    /// \brief Subscriber to camera image messages.
    private: transport::SubscriberPtr infoSub;

    private: QPushButton *pbSpawn;
    private: QPushButton *pbCalibrate;
    private: QLabel *lbName;
    private: QComboBox *cbType;
    private: QComboBox *cbFun;
    private: QDoubleSpinBox *sbC1;
    private: QDoubleSpinBox *sbC2;
    private: QDoubleSpinBox *sbC3;
    private: QDoubleSpinBox *sbF;
    private: QDoubleSpinBox *sbCA;
    private: QCheckBox *cbCircular;

    /// \brief Image output frame.
    // private: gui::ImageFrame *imgFrame;
  };
}
#endif
