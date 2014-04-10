/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GUI_ACTIONS_HH_
#define _GUI_ACTIONS_HH_

#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    extern QAction *g_newAct;
    extern QAction *g_openAct;
    extern QAction *g_importAct;
    extern QAction *g_saveAct;
    extern QAction *g_saveAsAct;
    extern QAction *g_saveCfgAct;
    extern QAction *g_aboutAct;
    extern QAction *g_quitAct;

    extern QAction *g_dataLoggerAct;

    extern QAction *g_newModelAct;
    extern QAction *g_resetModelsAct;
    extern QAction *g_resetWorldAct;
    extern QAction *g_editBuildingAct;
    extern QAction *g_editTerrainAct;
    extern QAction *g_editModelAct;

    extern QAction *g_playAct;
    extern QAction *g_pauseAct;
    extern QAction *g_stepAct;

    extern QAction *g_boxCreateAct;
    extern QAction *g_sphereCreateAct;
    extern QAction *g_cylinderCreateAct;
    extern QAction *g_meshCreateAct;
    extern QAction *g_pointLghtCreateAct;
    extern QAction *g_spotLghtCreateAct;
    extern QAction *g_dirLghtCreateAct;

    extern QAction *g_screenshotAct;

    extern QAction *g_showCollisionsAct;
    extern QAction *g_showGridAct;
    extern QAction *g_showContactsAct;
    extern QAction *g_showJointsAct;
    extern QAction *g_showCOMAct;
    extern QAction *g_transparentAct;

    extern QAction *g_resetAct;
    extern QAction *g_fullScreenAct;
    extern QAction *g_fpsAct;
    extern QAction *g_orbitAct;

    extern QAction *g_arrowAct;
    extern QAction *g_translateAct;
    extern QAction *g_rotateAct;
    extern QAction *g_scaleAct;

    extern QAction *g_topicVisAct;

    extern QAction *g_diagnosticsAct;

    extern QAction *g_viewWireframeAct;

    /// \class DeleteAction Actions.hh gui/gui.hh
    /// \brief Custom delete action.
    class GAZEBO_VISIBLE DeleteAction : public QAction
    {
      Q_OBJECT
      /// \brief Constructor
      /// \param[in] _text Name of the action.
      /// \param[in] _parent Action parent object.
      public: DeleteAction(const QString &_text, QObject *_parent)
              : QAction(_text, _parent) {}

      /// \brief Emit the delete signal. This triggers the action.
      /// \param[in] _modelName Name of the model to delete.
      public: void Signal(const std::string &_modelName)
               { emit DeleteSignal(_modelName); }

      /// \brief The custom signal which a SLOT can connect to.
      /// \param[in] _modelName The name of the model to delete.
      Q_SIGNALS: void DeleteSignal(const std::string &_modelName);
    };

    /// \brief Action used to delete a model.
    extern DeleteAction *g_deleteAct;
  }
}
#endif
