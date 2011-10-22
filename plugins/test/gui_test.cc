/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <boost/bind.hpp>

#include "gui/Gui.hh"
#include "rendering/Scene.hh"
#include "rendering/Camera.hh"
#include "rendering/DepthCamera.hh"
#include "rendering/RenderEngine.hh"
#include "rendering/UserCamera.hh"
#include "rendering/GUIOverlay.hh"
#include "gazebo.h"

namespace gazebo
{
  class GUITest : public GUIPlugin
  {
    public: void Load()
    {

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->connections.push_back( 
          event::Events::ConnectPreRender( 
            boost::bind(&GUITest::PreRender, this) ) );

      this->verbs.push_back( "Move" );
      this->verbs.push_back( "Pick" );
      this->verbs.push_back( "Put" );
      this->verbs.push_back( "Open" );
      this->verbs.push_back( "Close" );

      this->prepositions.push_back( "to" );
      this->prepositions.push_back( "up" );
      this->prepositions.push_back( "down" );
      this->prepositions.push_back( "on" );
      this->prepositions.push_back( "in" );

      this->nouns.push_back( "table" );
      this->nouns.push_back( "rice" );
      this->nouns.push_back( "salt" );
      this->nouns.push_back( "pepper" );
      this->nouns.push_back( "wine" );
      this->nouns.push_back( "mushrooms" );
    }

    private: void PreRender()
             {
               static bool connected = false;

               if (!connected)
               {
                 rendering::UserCameraPtr userCam = gui::get_active_camera();
                 rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene("default");
                 if (!scene)
                   gzerr << "Unable to find scene[default]\n";

                 this->camera = scene->CreateCamera("POV");
                 this->camera->Load();
                 this->camera->Init();
                 this->camera->SetClipDist(0.1, 20);
                 this->camera->SetCaptureData(true);
                 this->camera->SetImageWidth(500);
                 this->camera->SetImageHeight(500);
                 this->camera->SetFOV( 1.5707 );
                 this->camera->CreateRenderTexture("POV_RTT");
                 this->camera->SetWorldPosition(math::Vector3(1.42, 1.16, 1.0));
                 this->camera->SetWorldRotation(math::Quaternion(0, 0, -.5235));

                 this->overheadCamera = scene->CreateCamera("Overhead");
                 this->overheadCamera->Load();
                 this->overheadCamera->Init();
                 this->overheadCamera->SetClipDist(0.1, 20);
                 this->overheadCamera->SetCaptureData(true);
                 this->overheadCamera->SetImageWidth(500);
                 this->overheadCamera->SetImageHeight(500);
                 this->overheadCamera->SetFOV( 1.5707 );
                 this->overheadCamera->CreateRenderTexture("Overhead_RTT");
                 this->overheadCamera->SetWorldPosition(math::Vector3(0, 0, 10.0));
                 this->overheadCamera->SetWorldRotation(math::Quaternion(0, 1.5707, 0));


                 userCam->GetGUIOverlay()->AttachCameraToImage( this->overheadCamera, 
                     "Root/CameraView");

                 userCam->GetGUIOverlay()->AttachCameraToImage( 
                     this->camera, 
                     "Root/OverheadView");

                 userCam->GetGUIOverlay()->ButtonCallback( 
                     "Root/PrepositionButton", 
                     &GUITest::OnPrepositionButton, this  );

                 userCam->GetGUIOverlay()->ButtonCallback( 
                     "Root/VerbButton", &GUITest::OnVerbButton, this  );

                 userCam->GetGUIOverlay()->ButtonCallback( 
                     "Root/NounButton", &GUITest::OnNounButton, this  );

                 userCam->GetGUIOverlay()->ButtonCallback( 
                     "Root/SendButton", &GUITest::OnSendButton, this  );

                 userCam->GetGUIOverlay()->GetWindow("Root/VerbList")->hide();
                 userCam->GetGUIOverlay()->GetWindow("Root/PrepositionList")->hide();
                 userCam->GetGUIOverlay()->GetWindow("Root/NounList")->hide();

                 userCam->GetGUIOverlay()->GetWindow("Root/VerbList")->subscribeEvent( CEGUI::Listbox::EventSelectionChanged, CEGUI::Event::Subscriber( &GUITest::OnVerbSelect, this) );

                 userCam->GetGUIOverlay()->GetWindow("Root/PrepositionList")->subscribeEvent( CEGUI::Listbox::EventSelectionChanged, CEGUI::Event::Subscriber( &GUITest::OnPrepositionSelect, this) );

                userCam->GetGUIOverlay()->GetWindow("Root/NounList")->subscribeEvent( CEGUI::Listbox::EventSelectionChanged, CEGUI::Event::Subscriber( &GUITest::OnNounSelect, this) );


                 connected = true;
               }
             }

    private: CEGUI::Window *GetWindow(const std::string &_name)
             {
               rendering::UserCameraPtr userCam = gui::get_active_camera();
               return userCam->GetGUIOverlay()->GetWindow(_name);
             }

    private: CEGUI::AnimationInstance* AnimateWindow( 
                 const std::string &_windowName,
                 const std::string &_animName, bool _start = true)
             {
               CEGUI::Window *win;
               CEGUI::Animation *anim;
               CEGUI::AnimationInstance *instance;
               CEGUI::AnimationManager *animManager;

               animManager = CEGUI::AnimationManager::getSingletonPtr();

               win = this->GetWindow(_windowName);
               anim = animManager->getAnimation(_animName);
               instance = animManager->instantiateAnimation(anim);
               instance->setTargetWindow(win);

               if (_start)
                 instance->start();

               return instance;
             }

    private: bool OnNounSelect(const CEGUI::EventArgs& _e)
             {
               this->OnSelect("Root/NounList", 
                   "Root/PrepositionButton", 
                   "Root/NounButton",
                   "Root/SendButton", 
                   ""
                   );

               CEGUI::ListboxItem *item = ((CEGUI::Listbox*)this->GetWindow("Root/NounList"))->getFirstSelectedItem();

               this->noun = item->getText().c_str();
               this->UpdateCommand();
               return true;
             }


    private: bool OnPrepositionSelect(const CEGUI::EventArgs& _e)
             {
               this->OnSelect("Root/PrepositionList", 
                   "Root/VerbButton", 
                   "Root/PrepositionButton", 
                   "Root/NounButton",
                   "Root/SendButton"
                   );

               CEGUI::ListboxItem *item = ((CEGUI::Listbox*)this->GetWindow("Root/PrepositionList"))->getFirstSelectedItem();

               this->preposition = item->getText().c_str();
               this->UpdateCommand();

               return true;
             }
    private: bool OnVerbSelect(const CEGUI::EventArgs& _e)
             {
               this->OnSelect("Root/VerbList", 
                   "", 
                   "Root/VerbButton", 
                   "Root/PrepositionButton",
                   "Root/NounButton"
                   );
               CEGUI::ListboxItem *item = ((CEGUI::Listbox*)this->GetWindow("Root/VerbList"))->getFirstSelectedItem();

               this->verb = item->getText().c_str();
               this->UpdateCommand();
               return true;
             }


    private: void UpdateCommand()
             {
               std::string txt = "Command:  ";
               txt += this->verb + "  ";
               txt += this->preposition + "  ";
               txt += this->noun;

               this->GetWindow("Root/Sentence")->setText(txt);
             }


    private: void OnSelect(const std::string &_name, 
                           const std::string &_leftButton, 
                           const std::string &_midButton, 
                           const std::string &_rightButton,
                           const std::string &_farRightButton)
             {
               CEGUI::AnimationInstance *instance, *instance1;

               instance = this->AnimateWindow(_name, "scroll_down");

               if (!_leftButton.empty())
               {
                 instance1 = this->AnimateWindow( _leftButton,
                                                  "move_left_4", false);

                 this->GetWindow(_name)->subscribeEvent( 
                     CEGUI::AnimationInstance::EventAnimationEnded, 
                     CEGUI::Event::Subscriber(
                       &CEGUI::AnimationInstance::handleStart, instance1) );

                 this->GetWindow(_leftButton)->disable();
               }

               instance1 = this->AnimateWindow(_midButton,
                                               "move_left_1", false);
               this->GetWindow(_name)->subscribeEvent( 
                   CEGUI::AnimationInstance::EventAnimationEnded, 
                   CEGUI::Event::Subscriber(
                     &CEGUI::AnimationInstance::handleStart, instance1) );
               this->GetWindow(_midButton)->disable();

               if (!_rightButton.empty())
               {
                 instance1 = this->AnimateWindow(_rightButton,
                     "move_left_2", false);
                 this->GetWindow(_name)->subscribeEvent( 
                     CEGUI::AnimationInstance::EventAnimationEnded, 
                     CEGUI::Event::Subscriber(
                       &CEGUI::AnimationInstance::handleStart, instance1) );
                 this->GetWindow(_rightButton)->enable();
               }

               if (!_farRightButton.empty())
               {
                 instance1 = this->AnimateWindow(_farRightButton,
                     "move_left_3", false);
                 this->GetWindow(_name)->subscribeEvent( 
                     CEGUI::AnimationInstance::EventAnimationEnded, 
                     CEGUI::Event::Subscriber(
                       &CEGUI::AnimationInstance::handleStart, instance1) );
                 this->GetWindow(_farRightButton)->disable();
               }
             }

    private: void OnSendButton()
             {
               printf("Send command to robot\n");
             }
 
    private: void OnNounButton()
             {
               this->OnButton("Root/NounList",0, 0, 0.32, this->nouns);
             }

    private: void OnPrepositionButton()
             {
               this->OnButton("Root/PrepositionList",0, 0.32,0, 
                              this->prepositions);
             }

    private: void OnVerbButton()
             {
               this->OnButton("Root/VerbList", 0.32,0, 0, this->verbs);
             }

    private: void OnButton(const std::string &_name, 
                           float _r, float _g, float _b,
                           const std::vector<std::string> &items)
             {
               CEGUI::ListboxTextItem *item;
               CEGUI::Listbox *win = (CEGUI::Listbox*)(this->GetWindow(_name));

               win->show();
               win->resetList();

               for (unsigned int i=0; i < items.size(); i++)
               {
                 item = new CEGUI::ListboxTextItem(items[i]);
                 item->setSelectionColours(CEGUI::colour(_r, _g, _b, 0.8));
                 item->setSelectionBrushImage( "Gazebo-Images", "FrameTop" );
                 win->addItem(item);
               }
               this->AnimateWindow(_name, "scroll_up");
             }


    private: void Init()
    {
      rendering::UserCameraPtr userCam = gui::get_active_camera();
      if (userCam && userCam->GetGUIOverlay())
      {
        userCam->GetGUIOverlay()->LoadLayout( "gui_test.layout" );
      }
    }

    private: transport::NodePtr node;
    private: std::vector<event::ConnectionPtr> connections;
    private: rendering::CameraPtr camera, overheadCamera;

    private: std::vector< std::string > verbs;
    private: std::vector< std::string > prepositions;
    private: std::vector< std::string > nouns;

    private: std::string verb;
    private: std::string preposition;
    private: std::string noun;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_GUI_PLUGIN(GUITest)
}
 
