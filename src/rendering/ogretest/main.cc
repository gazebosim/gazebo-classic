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
// sample example

// actually, you can just include specifics here
// instead of all of ogre, but this makes it easier
// on me and this example
#include <Ogre.h>
// OIS is a bit lighting, though.  just include all of it
#include <OIS/OIS.h>

// for srand()
#include <cstdlib>
// for time()
#include <ctime>

// config files give us "stuff", not just stuff
// we must manually remove the quotes
Ogre::String removeQuotes(const Ogre::String &str) {
    Ogre::String nstr = "";
    for (size_t i = 0; i < str.size(); ++i) {
        if (str[i] == '\"' || str[i] == '\'')
            continue;
        nstr += str[i];
    }
    return nstr;
}

// I don't really suggest you keep globals like
// this, but, again, simplistic example
class Mgr: public Ogre::WindowEventListener {
  public:
    // a config file we'll use to retrieve some settings
    Ogre::ConfigFile cfgFile;

    // our root
    Ogre::Root *root;

    // simples stuff here
    Ogre::SceneManager *sceneMgr;
    Ogre::Camera *cam;
    Ogre::Viewport *vp;
    Ogre::RenderWindow *window;
    // note we only used one camera & viewport, simplistic
    Ogre::Timer *timer; // we'll just use the root's timer
    // otherwise you can make more complex timer systems

    // just incase
    Ogre::RenderSystem *rsys;

    // OIS stuff O_o
    OIS::InputManager *input;
    OIS::Keyboard *keys;
    OIS::Mouse *mouse;

    // IGNORE THIS FOR NOW, until "initShadows".  capish?
    void shadowTextureCasterPreViewProj(Ogre::Light *light, Ogre::Camera *cam);
    // these are pure virtual but we don't need them...  so just make them empty
    // otherwise we get "cannot declare of type Mgr due to missing abstract
    // functions" and so on
    void shadowTexturesUpdated(size_t) {}
    void shadowTextureReceiverPreViewProj(Ogre::Light*, Ogre::Frustum*) {}
    // local "continue?" variable
    bool running;

    // this tells us if the window closed (or alt-f4, for example)
    void windowClosed(Ogre::RenderWindow *window) {
        // stop running.  we don't compare any windows
        // since we only make 1 window anyways
        running = false;
    }

    Mgr():
      root(NULL), sceneMgr(NULL), cam(NULL), vp(NULL),
      window(NULL), timer(NULL), rsys(NULL), input(NULL), keys(NULL),
      mouse(NULL), running(true) {
        // stupid constructor.  don't base your code on this code ;)
        // but since I wanted to make it easy, everything is done
        // in seperate "init" functions.  so this just makes everything
        // default
    }
} mgr;

// simple "choosing" of the renderer
enum Renderer { D3D, OGL };
// initialize Ogre and the renderer
void initRenderer() { // default to D3D
    // create our Root
    // note the ""s.  we supply our own plugins and config options
    mgr.root = new Ogre::Root("", ""); // default "Ogre.log"
    mgr.root->loadPlugin("/home/nate/local/lib/OGRE/RenderSystem_GL.so");
    mgr.root->loadPlugin(
        "/home/nate/local/lib/OGRE/Plugin_CgProgramManager.so");

    // get the available render systems
    // technically, should be only one since we only load one plugin
    // much better than comparing the string names like some people do it
    Ogre::RenderSystemList renderers = mgr.root->getAvailableRenderers();

    // use the first one
    // (the '*' is because begin() returns an iterator, like the STL)
    mgr.rsys = *(renderers.begin());
    // tell the root to use this render system
    mgr.root->setRenderSystem(mgr.rsys);

    // now we can initialise the root
    // though pass false since we want our own window creation
    mgr.root->initialise(false);

    // OK, this is really badly hard coded.  but its an example
    // the disorder engine, for example, reads a lua script for these values
    // (the disorder engine is what's under Portalized)
    // (*cough*don'tworryifyou'veneverheardofit, it'smyproject*cough*)
    Ogre::NameValuePairList miscParams;
    // retrive anti aliasing from the config file
    miscParams["FSAA"] =
      removeQuotes(mgr.cfgFile.getSetting("FSAA", "renderer"));
    miscParams["vsync"] =
      removeQuotes(mgr.cfgFile.getSetting("vsync", "renderer"));

    // get these settings from the cfg file, too
    int width = 800, height = 600;
    bool fullscreen = false;

    std::string tempData = mgr.cfgFile.getSetting("width", "renderer");
    width = Ogre::StringConverter::parseInt(removeQuotes(tempData));
    tempData = mgr.cfgFile.getSetting("height", "renderer");
    height = Ogre::StringConverter::parseInt(removeQuotes(tempData));
    tempData = mgr.cfgFile.getSetting("fullscreen", "renderer");
    fullscreen = Ogre::StringConverter::parseBool(removeQuotes(tempData));

    // now we can just make a window, it's that easy
    mgr.window = mgr.root->createRenderWindow(
        "Ogre Soft Shadowing", width, height, fullscreen, &miscParams);

    // btw, if my comments seem weird or something,
    // it probably has to do with the fact that I'm listening
    // to 50 cent right now, rocking with the awesome rap
    // @ hatas! don't you hate 50 ;).  he's coo1.  (in da club, anyone?)

    // ok, back to coding

    // now, we can, for example, set the default texture filtering
    ::Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(
        ::Ogre::TFO_ANISOTROPIC);
    // tada.

    // tell the window it's active (duh, rofl)
    mgr.window->setActive(true);
    // make it auto updated on renderOneFrame()
    mgr.window->setAutoUpdated(true);

    // time to prepare the scene
    mgr.sceneMgr = mgr.root->createSceneManager(Ogre::ST_GENERIC);
    // just generic for the example
    // the shadows will work on anything that follows ogre materials
    // properly, BTW
    // "some" ambient light
    mgr.sceneMgr->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));

    // alright, the next bit of code I'll just copy & paste from my project

    // make the camera
    mgr.cam = mgr.sceneMgr->createCamera("Mgr::cam");
    mgr.cam->setNearClipDistance(0.01);
    mgr.cam->setFarClipDistance(1000);

    // add a viewport to the window and this camera
    // (covers up the whole window)
    mgr.vp = mgr.window->addViewport(mgr.cam);
    // black background
    mgr.vp->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0));
    // clear every frame for us
    mgr.vp->setClearEveryFrame(true);

    // aspect ratio, etc.
    mgr.cam->setAspectRatio(mgr.window->getWidth() /
        float(mgr.window->getHeight()));
    // 60 degree FOV
    mgr.cam->setFOVy(Ogre::Degree(60));

    // set the timer
    mgr.timer = mgr.root->getTimer();

    // we're done with Ogre itself
}

void stopRenderer() {
    // the scene manager being destroyed will in turn
    // handle the camera/viewport
    mgr.root->destroySceneManager(mgr.sceneMgr);

    // deleting the root will take care of the window and the root itself
    delete mgr.root;
    // actually, deleting the root shoot take care of the scene manager
    // for us automatically.  but I just wanted to manually show it here.
}

void initResources() {
    // just some hard coded stuff
    Ogre::ResourceGroupManager *rgm =
      Ogre::ResourceGroupManager::getSingletonPtr();

    // add our data directory (we sort of stuff everything in there
    // just to make it easy on ourselves.  organize better otherwise!)
    rgm->addResourceLocation("../data", "FileSystem", "Media");
    // init this group
    rgm->initialiseResourceGroup("Media"); // "Media" is the group from above
}

void initShadows() {
    // we'll be self shadowing
    mgr.sceneMgr->setShadowTextureSelfShadow(true);

    // our caster material
    mgr.sceneMgr->setShadowTextureCasterMaterial("shadow_caster");
    // note we have no "receiver".  all the "receivers" are integrated.

    // get the shadow texture count from the cfg file
    std::string tempData = mgr.cfgFile.getSetting("shadowTextureCount",
        "renderer");

    // (each light needs a shadow texture)
    mgr.sceneMgr->setShadowTextureCount(Ogre::StringConverter::parseInt(
          removeQuotes(tempData)));

    // the size, too (1024 looks good with 3x3 or more filtering)
    tempData = mgr.cfgFile.getSetting("shadowTextureRes", "renderer");
    mgr.sceneMgr->setShadowTextureSize(Ogre::StringConverter::parseInt(
          removeQuotes(tempData)));

    // float 16 here.  we need the R and G channels.
    // float 32 works a lot better with a low/none VSM epsilon
    // (wait till the shaders)
    // but float 16 is "good enough" and supports bilinear filtering
    // on a lot of cards
    // (note we're using floats and not bytes.  bytes, 0-255, won't be
    // able to store our depth data accurately enough.  technically,
    // we can "split" the floats into multiple bytes ourselves, but it's
    // rather "complicated", to put it simply)
    mgr.sceneMgr->setShadowTexturePixelFormat(Ogre::PF_FLOAT16_RGB);

    // big NONO to render back faces for VSM.  it doesn't need any biasing
    // so it's worthless (and rather problematic) to use the back face hack
    // that
    // works so well for normal depth shadow mapping (you know, so you don't
    // get surface acne)
    mgr.sceneMgr->setShadowCasterRenderBackFaces(false);

    const unsigned numShadowRTTs = mgr.sceneMgr->getShadowTextureCount();
    for (unsigned i = 0; i < numShadowRTTs; ++i) {
        Ogre::TexturePtr tex = mgr.sceneMgr->getShadowTexture(i);
        Ogre::Viewport *vp =
          tex->getBuffer()->getRenderTarget()->getViewport(0);
        vp->setBackgroundColour(Ogre::ColourValue(1, 1, 1, 1));
        vp->setClearEveryFrame(true);
        Ogre::CompositorManager::getSingleton().addCompositor(vp, "blur");
        Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp,
            "blur", true);
    }

    // enable integrated additive shadows
    // actually, since we render the shadow map ourselves, it doesn't
    // really matter whether they are additive or modulative
    // as long as they are integrated v(O_o)v
    mgr.sceneMgr->setShadowTechnique(
        Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);

    // some "tests" with shadow camera setups
    //Ogre::LiSPSMShadowCameraSetup *camSetup =
    //new Ogre::LiSPSMShadowCameraSetup();
    //Ogre::FocusedShadowCameraSetup *camSetup =
    //new Ogre::FocusedShadowCameraSetup();
    //mgr.sceneMgr->setShadowCameraSetup(Ogre::ShadowCameraSetupPtr(camSetup));

    // all done for the shadows.  see how easy it was?  might want to experiment
    // with stuff like different shadow camera setups.  or something.
}

void Mgr::shadowTextureCasterPreViewProj(Ogre::Light *light, Ogre::Camera *cam)
{
    // basically, here we do some forceful camera near/far clip attenuation
    // yeah.  simplistic, but it works nicely.  this is the function I
    // was talking
    // about you ignoring above in the Mgr declaration.
    float range = light->getAttenuationRange();
    cam->setNearClipDistance(range * 0.01);
    cam->setFarClipDistance(range);
    // we just use a small near clip so that the light doesn't "miss" anything
    // that can shadow stuff.  and the far clip is equal to the lights' range.
    // (thus, if the light only covers 15 units of objects, it can only
    // shadow 15 units - the rest of it should be attenuated away,
    // and not rendered)
}

void initInput() {
    // aRGHh I forgot about input -_-

    // window handle (IIRC, this was cross-platform)
    size_t winHandle;
    mgr.window->getCustomAttribute("WINDOW", &winHandle);

    // create input "manager" or "hook", as I call it
    mgr.input = OIS::InputManager::createInputSystem(winHandle);

    mgr.keys = (OIS::Keyboard*)mgr.input->createInputObject(OIS::OISKeyboard,
        true);
    mgr.mouse = (OIS::Mouse*)mgr.input->createInputObject(OIS::OISMouse, true);

    // tada.  I think.  rofl just kidding.  seriously.  that's it.
}

void stopInput() {
    // destroy the keyboard object, the mouse object
    // and the input "hook"
    mgr.input->destroyInputObject(mgr.keys);
    mgr.input->destroyInputObject(mgr.mouse);
    OIS::InputManager::destroyInputSystem(mgr.input);
}

void captureInput() {
    // we have to "capture" the current "state" of the input devices
    // every single frame.  (think of it as an "input update()")
    mgr.keys->capture();
    mgr.mouse->capture();
}

// handling the camera every frame based on a delta time and mouse/keys
void handleCamera(float dt) {
    // this is actually a bad way to do a first person camera
    // however, I'm trying to keep this relatively simple.
    static float pitch = 0, yaw = 0;

    // get the current mouse state
    const OIS::MouseState &ms = mgr.mouse->getMouseState();
    // fix pitch and yaw
    pitch += -ms.Y.rel;// add the relative mouse movement (up/down for pitching)
    yaw += -ms.X.rel; // add the relative mouse movement (left/right for yawing)
    // constrain pitch to +/- 90 degrees
    if (pitch < -90) pitch = -90;
    if (pitch >  90) pitch =  90;

    // set the camera to a default orientation and then get
    // it into our own rotation
    mgr.cam->setOrientation(Ogre::Quaternion::IDENTITY);
    // yaw first, pitch second (all in world space)
    mgr.cam->yaw(Ogre::Degree(yaw));
    mgr.cam->pitch(Ogre::Degree(pitch));

    // define a local movement vector (+X == right, +Y == yp, -Z == forward)
    Ogre::Vector3 move(0, 0, 0);
    // basic WASD.  for your own entertainment, don't hardcode like me ;)
    if (mgr.keys->isKeyDown(OIS::KC_W))
        move.z += -1;
    if (mgr.keys->isKeyDown(OIS::KC_S))
        move.z += 1;
    if (mgr.keys->isKeyDown(OIS::KC_A))
        move.x += -1;
    if (mgr.keys->isKeyDown(OIS::KC_D))
        move.x += 1;
    // move the camera based on where it's looking at a speed
    const float CAM_SPEED = 10; // meters per second
    mgr.cam->move(mgr.cam->getOrientation() * (move * CAM_SPEED * dt));
}

Ogre::Quaternion makeQuat(
    const Ogre::Vector3 &forward, const Ogre::Vector3 &wantedUp =
    Ogre::Vector3::UNIT_Y) {
    // if you don't understand this, don't worry
    // just basically makes a full orientation based
    // on a desired "up" vector and a desired "forward" vector
    Ogre::Vector3 right = wantedUp.crossProduct(forward).normalisedCopy();
    Ogre::Vector3 up = forward.crossProduct(right).normalisedCopy();
    return Ogre::Quaternion(right, up, forward);
}

void setupScene() {
    // create a simply "plane" mesh programmatically
    Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createPlane(
        "plane.mesh", // simulate actual ".mesh", rofl
        "Media", // in the Media group
        Ogre::Plane(Ogre::Vector3::UNIT_Y, 0), // plane facing up
        25, 25, // world coordinates.  we want it nice and big
        1, 1, // number of segments...  might want to tesselate it if you wish
        true, // yes, we need normals for proper lighting
        1, // one texture coordinate set
        1, 1, // UV tiling (texture coordinates)
        Ogre::Vector3::NEGATIVE_UNIT_Z); // up vector.
    //  if it's facing up, then it's up is -Z
    // make sure it loaded
    planeMesh->load();

    Ogre::SceneNode *rootNode = mgr.sceneMgr->getRootSceneNode();

    // make an entity for this plane
    Ogre::Entity *ent = mgr.sceneMgr->createEntity("planeEnt", "plane.mesh");
    ent->setMaterialName("metal");
    // make a node for it, scale to 10 by 10 meters
    Ogre::SceneNode *node = rootNode->createChildSceneNode();
    // attach
    // EDIT: since we're using a level, no need to make the plane
    // (so actually don't attach)
    node->attachObject(ent);

    // create some random knots
    const size_t NUM_X = 5, NUM_Z = NUM_X;
    for (size_t x = 0; x < NUM_X; ++x) {
        for (size_t z = 0; z < NUM_Z; ++z) {
            Ogre::Vector3 p(
                Ogre::Math::RangeRandom(-5.0, 5.0),
                Ogre::Math::RangeRandom(0.25, 2),
                Ogre::Math::RangeRandom(-5.0, 5.0));
            ent = mgr.sceneMgr->createEntity(
                "ogre" + Ogre::StringConverter::toString(x) + "_" +
                Ogre::StringConverter::toString(z), "knot.mesh");
            ent->setMaterialName("ogre");
            node = rootNode->createChildSceneNode();
            node->attachObject(ent);
            node->setPosition(p);
            node->setScale(Ogre::Vector3(0.01, 0.01, 0.01));
        }
    }
    /*ent = mgr.sceneMgr->createEntity("level", "level.mesh");
    ent->setMaterialName("ogre");
    node = rootNode->createChildSceneNode();
    node->attachObject(ent);
    node->setScale(Ogre::Vector3(0.01, 0.01, 0.01)); // from cm to m
    */

    // random position I found to be pretty
    mgr.cam->setPosition(Ogre::Vector3(0.0361507, 2.70619, 7.03829));
    mgr.cam->setDirection(Ogre::Vector3(0, 0, -1));

    // create our flash light
    Ogre::Light *light = mgr.sceneMgr->createLight("flashLight");
    light->setDiffuseColour(Ogre::ColourValue(1, 1, 1));
    light->setType(Ogre::Light::LT_SPOTLIGHT);
    light->setSpotlightInnerAngle(Ogre::Degree(45));
    light->setSpotlightOuterAngle(Ogre::Degree(65));
    light->setAttenuation(50, 1, 1, 1); // meter range.
    light->setDirection(Ogre::Vector3(0, 0, -1));
    node = rootNode->createChildSceneNode("flashLightNode");
    node->attachObject(light);
}

void handleScene(float dt) {
    // sync our "flashlight" with the camera
    Ogre::SceneNode *node = mgr.sceneMgr->getSceneNode("flashLightNode");
    node->setPosition(mgr.cam->getDerivedPosition());
    node->setOrientation(mgr.cam->getDerivedOrientation());
    // this is a simply "offset", to the bottom right, as if
    // the camera is holding the light in its right hand
    Ogre::Vector3 offset(0.225, -0.3, -0.3);
    // local space means that it's relative to the node
    // itself, not in world units
    node->translate(offset, Ogre::Node::TS_LOCAL);
}

// example of a key listener
struct LightControl: public OIS::KeyListener {
    LightControl(size_t li = 0): lightIndex(li) {}
    size_t lightIndex;

    bool keyPressed(const OIS::KeyEvent &e) {
        // place a light if we pressed space
        if (e.key == OIS::KC_SPACE) {
            std::string name = "light" +
              Ogre::StringConverter::toString(lightIndex++);
            Ogre::Light *light = mgr.sceneMgr->createLight(name);
            // random colour
            float r = Ogre::Math::UnitRandom(),
                g = Ogre::Math::UnitRandom(),
                b = Ogre::Math::UnitRandom();
            // make sure the light is not too dark
            if (Ogre::Vector3(r, g, b).length() < 0.75) {
                // if so, multiply by 2
                r *= 2;
                g *= 2;
                b *= 2;
            }
            light->setDiffuseColour(Ogre::ColourValue(r, g, b));

            light->setDirection(Ogre::Vector3(0, -1, 0));

            light->setType(Ogre::Light::LT_POINT);
            //light->setSpotlightInnerAngle(Ogre::Degree(70));
            //light->setSpotlightOuterAngle(Ogre::Degree(90));

            light->setAttenuation(30, 1, 1, 1);

            Ogre::SceneNode *node =
                mgr.sceneMgr->getRootSceneNode()->createChildSceneNode(
                    name + "Node");
            node->setPosition(mgr.cam->getDerivedPosition());
            node->setOrientation(mgr.cam->getDerivedOrientation());
            node->attachObject(light);
        }

        // toggle the flash light if we press F
        if (e.key == OIS::KC_F) {
            Ogre::Light *flashLight = mgr.sceneMgr->getLight("flashLight");
            flashLight->setVisible(!flashLight->getVisible());
        }

        return true;
    }

    bool keyReleased(const OIS::KeyEvent &e) {
        // do nothing
        return true;
    }
} lightPlacer(0);

int main(int, char**) {
    // seed the random number generator
    std::srand(std::time(NULL));

    // load our config file
    mgr.cfgFile.loadDirect("../data/config.ini");
    // get our renderer (OpenGL or Direct3D)
    removeQuotes(mgr.cfgFile.getSetting("library", "renderer"));

    // start up the renderer and a window (OGL for the example)
    initRenderer();
    // load all of our shaders and models and stuff
    initResources();
    // init the shadow setup we'll be using
    initShadows();
    // grab the window's input stuff with OIS
    initInput();
    // grab the keyboard input with our light placer
    mgr.keys->setEventCallback(&lightPlacer);

    // set up our basic scene
    setupScene();

    // this keeps track of time (seconds to keep it easy)
    float currentTime = float(mgr.timer->getMilliseconds()) * 0.001;

    // keep running until something stops us
    // (frame listener, escape key, window closed
    // (the mgr gets a message for this), etc.)
    for (bool running = true; running; running = running && mgr.running) {
        // note this
        // when we render manually, we need to pump
        // the window events ourselves
        // (or we won't get window events, like closing of the window,
        // input)
        Ogre::WindowEventUtilities::messagePump();

        // get the delta time and advance currentTime
        float deltaTime =
          (float(mgr.timer->getMilliseconds()) * 0.001) - currentTime;
        currentTime += deltaTime;

        // capture our input state
        captureInput();

        // simply "escape" key exit
        if (mgr.keys->isKeyDown(OIS::KC_ESCAPE))
            running = false;

        // handle the camera movement
        handleCamera(deltaTime);

        // handle the scene movement (like animations for the robot)
        handleScene(deltaTime);

        // render one frame by Ogre
        // technically, we can go even lower-level and update the window RT
        // ourselves.  but that needs tweaking for frame listeners and what-not
        running = running && mgr.root->renderOneFrame();
        // BTW, renderOneFrame() returns true if everything went alright
        // (such as frame listeners, which we actually don't use here)
    }

    // destroy input
    stopInput();
    // stop and deinitialise Ogre
    stopRenderer();
}
