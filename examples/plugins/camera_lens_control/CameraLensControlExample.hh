/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _CAMERA_LENS_CONTROL_EXAMPLE_HH_
#define _CAMERA_LENS_CONTROL_EXAMPLE_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE CameraLensControlExample : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent widget
    public: CameraLensControlExample();

    /// \brief Destructor
    public: virtual ~CameraLensControlExample();

    protected: virtual void LoadGUIComponents(QWidget *_parent);

    /// \brief Callbacks for QUI events
    protected slots: void OnButtonSpawn();

    protected slots: void OnButtonCalibrate();

    protected slots: void OnCbTypeChange();

    protected slots: void OnCbFunChange();  

    protected slots: void OnSbChange();

    protected slots: void OnValueChanged();

    protected slots: void OnCustomRequested();

    protected slots: void OnTypeChanged();

    /// \brief Callback for "~/selection" topic
    protected: void OnSelect(ConstSelectionPtr &_msg);

    /// \brief Callback for "~/*/wideanglecamera_*/link/image_stamped" topic
    protected: void OnImageUpdate(ConstImageStampedPtr &_msg);

    /// \brief Callback for "~/*/wideanglecamera_*/link/lens_info" topic
    protected: void OnCameraLensCmd(ConstCameraLensCmdPtr &_msg);

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

    /// \brief QT Widgets, that will be loaded from .ui file
    private: QPushButton *pbSpawn;
    private: QPushButton *pbCalibrate;
    private: QLabel *lbName;
    private: QComboBox *cbType;
    private: QComboBox *cbFun;
    private: QDoubleSpinBox *sbC1;
    private: QDoubleSpinBox *sbC2;
    private: QDoubleSpinBox *sbC3;
    private: QDoubleSpinBox *sbF;
    private: QDoubleSpinBox *sbHFOV;
    private: QDoubleSpinBox *sbCA;
    private: QCheckBox *cbScaleToHFOV;

    /// \brief Ignore messages from topic provider if it is set to false
    private: bool acceptInfoMessages = true;
  };
}
#endif
