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
#ifndef _GAZEBO_CAMERA_LENS_CONTROL_EXAMPLE_HH_
#define _GAZEBO_CAMERA_LENS_CONTROL_EXAMPLE_HH_

#include <string>

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

    // /// \
    // protected: void mouseMoveEvent(QMouseEvent *_event) override;
    // protected: void mousePressEvent(QMouseEvent *_event) override;
    // protected: void mouseReleaseEvent(QMouseEvent *_event) override;
    // protected: void mouseDoubleClickEvent(QMouseEvent *_event) override;
    protected: bool eventFilter(QObject *obj, QEvent *event) override;

    /// \brief Load GUI components
    /// \param[in] _parent Pointer to the parent widget
    private: virtual void LoadGUIComponents(QWidget *_parent);

    /// \brief Callback to spawn a button
    private slots: void OnButtonSpawn();

    /// \brief Callback triggered when a value has been changed.
    private slots: void OnValueChanged();

    /// \brief Callback when a type has been changed
    private slots: void OnTypeChanged();

    /// \brief Callback for "~/selection" topic
    /// \param[in] _msg The selection message
    private: void OnSelect(ConstSelectionPtr &_msg);

    /// \brief Callback for "~/*/wideanglecamera_*/link/lens_info" topic
    /// \param[in] _msg The camera lens command message
    private: void OnCameraLens(ConstCameraLensPtr &_msg);

    /// \brief Counter used to create unique model names.
    private: unsigned int counter = 0;

    // \brief Name of current element selected in gazebo.
    private: std::string selectedElementName;

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    /// \brief Publisher of factory messages.
    private: transport::PublisherPtr factoryPub;

    /// \brief Publisher of camera control messages.
    private: transport::PublisherPtr cameraControlPub;

    /// \brief Subscriber to selection messages.
    private: transport::SubscriberPtr selectionSub;

    /// \brief Subscriber to camera image messages.
    private: transport::SubscriberPtr infoSub;

    /// \brief Spawn button.
    private: QPushButton *pbSpawn;

    /// \brief Calibrate button.
    private: QPushButton *pbCalibrate;

    /// \brief Label name.
    private: QLabel *lbName;

    /// \brief Type combo box.
    private: QComboBox *cbType;

    /// \brief Function combo box.
    private: QComboBox *cbFun;

    /// \brief Spin box for lens C1 value.
    private: QDoubleSpinBox *sbC1;

    /// \brief Spin box for lens C2 value.
    private: QDoubleSpinBox *sbC2;

    /// \brief Spin box for lens C3 value.
    private: QDoubleSpinBox *sbC3;

    /// \brief Spin box for lens linear scaling factor.
    private: QDoubleSpinBox *sbF;

    /// \brief Spin box for horizontal field of view.
    private: QDoubleSpinBox *sbHFOV;

    /// \brief Spin box for cutoff angle.
    private: QDoubleSpinBox *sbCA;

    /// \brief Checkbox to scale to horizontal field of view.
    private: QCheckBox *cbScaleToHFOV;

    /// \brief Ignore messages from topic provider if it is set to false
    private: bool acceptInfoMessages = true;
  };
}
#endif
