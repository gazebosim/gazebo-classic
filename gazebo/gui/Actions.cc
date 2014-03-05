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

#include "gazebo/gui/Actions.hh"

QAction *gazebo::gui::g_arrowAct = 0;
QAction *gazebo::gui::g_translateAct = 0;
QAction *gazebo::gui::g_rotateAct = 0;
QAction *gazebo::gui::g_scaleAct = 0;
QAction *gazebo::gui::g_newAct = 0;
QAction *gazebo::gui::g_openAct = 0;
QAction *gazebo::gui::g_importAct = 0;
QAction *gazebo::gui::g_saveAct = 0;
QAction *gazebo::gui::g_saveAsAct = 0;
QAction *gazebo::gui::g_aboutAct = 0;
QAction *gazebo::gui::g_quitAct = 0;
QAction *gazebo::gui::g_dataLoggerAct = 0;


QAction *gazebo::gui::g_newModelAct = 0;
QAction *gazebo::gui::g_resetModelsAct = 0;
QAction *gazebo::gui::g_resetWorldAct = 0;
QAction *gazebo::gui::g_editBuildingAct = 0;
QAction *gazebo::gui::g_editTerrainAct = 0;

QAction *gazebo::gui::g_playAct = 0;
QAction *gazebo::gui::g_pauseAct = 0;
QAction *gazebo::gui::g_stepAct = 0;

QAction *gazebo::gui::g_boxCreateAct = 0;
QAction *gazebo::gui::g_sphereCreateAct = 0;
QAction *gazebo::gui::g_cylinderCreateAct = 0;
QAction *gazebo::gui::g_meshCreateAct = 0;
QAction *gazebo::gui::g_pointLghtCreateAct = 0;
QAction *gazebo::gui::g_spotLghtCreateAct = 0;
QAction *gazebo::gui::g_dirLghtCreateAct = 0;

QAction *gazebo::gui::g_screenshotAct = 0;

QAction *gazebo::gui::g_showCollisionsAct = 0;
QAction *gazebo::gui::g_showGridAct = 0;
QAction *gazebo::gui::g_showContactsAct = 0;
QAction *gazebo::gui::g_showJointsAct = 0;
QAction *gazebo::gui::g_showCOMAct = 0;
QAction *gazebo::gui::g_transparentAct = 0;
QAction *gazebo::gui::g_viewWireframeAct = 0;

QAction *gazebo::gui::g_resetAct = 0;
QAction *gazebo::gui::g_fullScreenAct = 0;
QAction *gazebo::gui::g_fpsAct = 0;
QAction *gazebo::gui::g_orbitAct = 0;

QAction *gazebo::gui::g_topicVisAct = 0;

QAction *gazebo::gui::g_diagnosticsAct = 0;

gazebo::gui::DeleteAction *gazebo::gui::g_deleteAct = 0;
