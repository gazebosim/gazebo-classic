#include <Ogre.h>
#include "OgreCreator.hh"
#include "MeshManager.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "SelectionObj.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SelectionObj::SelectionObj()
{
  this->node = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SelectionObj::~SelectionObj()
{
  if (this->node != NULL)
    OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->removeAndDestroyChild( this->node->getName() );
}

////////////////////////////////////////////////////////////////////////////////
// Load
void SelectionObj::Load()
{
  Ogre::SceneNode *pnode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode();
  this->node = pnode->createChildSceneNode("selection_node");

  Ogre::SceneNode *rotNode = this->node->createChildSceneNode("rot_node");
  Ogre::SceneNode *transNode = this->node->createChildSceneNode("trans_node");
  transNode->setInheritOrientation(false);
  

  if (!this->node->getCreator()->hasEntity("selection_tube"))
    OgreCreator::InsertMesh(MeshManager::Instance()->GetMesh("selection_tube"));

  Ogre::SceneNode *xTube, *yTube, *zTube, *xBox, *yBox, *zBox;

  xTube = rotNode->createChildSceneNode("selection_nodeX");
  xTube->setInheritScale(true);
  xTube->yaw(Ogre::Radian(M_PI/2.0));

  yTube = rotNode->createChildSceneNode("selection_nodeY");
  yTube->setInheritScale(true);
  yTube->pitch(Ogre::Radian(M_PI/2.0));

  zTube = rotNode->createChildSceneNode("selection_nodeZ");
  zTube->setInheritScale(true);

  xBox = transNode->createChildSceneNode("selection_transX");
  xBox->setInheritScale(false);

  yBox = transNode->createChildSceneNode("selection_transY");
  yBox->setInheritScale(false);

  zBox = transNode->createChildSceneNode("selection_transZ");
  zBox->setInheritScale(false);



  Ogre::MovableObject *tubeObj1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ1", "selection_tube"));
  tubeObj1->setCastShadows(false);
  tubeObj1->setUserAny( Ogre::Any(std::string("rotx")) );
  ((Ogre::Entity*)tubeObj1)->setMaterialName("Gazebo/Red");

  Ogre::MovableObject *tubeObj2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ2", "selection_tube"));
  tubeObj2->setCastShadows(false);
  tubeObj2->setUserAny( Ogre::Any(std::string("roty")) );
  ((Ogre::Entity*)tubeObj2)->setMaterialName("Gazebo/Green");

  Ogre::MovableObject *tubeObj3 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ3", "selection_tube"));
  tubeObj3->setCastShadows(false);
  tubeObj3->setUserAny( Ogre::Any(std::string("rotz")) );
  ((Ogre::Entity*)tubeObj3)->setMaterialName("Gazebo/Blue");

  Ogre::MovableObject *boxObj1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ4", "unit_box_U1V1"));
  boxObj1->setCastShadows(false);
  boxObj1->setUserAny( Ogre::Any(std::string("transx")) );
  ((Ogre::Entity*)boxObj1)->setMaterialName("Gazebo/Red");

  Ogre::MovableObject *boxObj2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ5", "unit_box_U1V1"));
  boxObj2->setCastShadows(false);
  boxObj2->setUserAny( Ogre::Any(std::string("transy")) );
  ((Ogre::Entity*)boxObj2)->setMaterialName("Gazebo/Green");

  Ogre::MovableObject *boxObj3 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ6", "unit_box_U1V1"));
  boxObj3->setCastShadows(false);
  boxObj3->setUserAny( Ogre::Any(std::string("transz")) );
  ((Ogre::Entity*)boxObj3)->setMaterialName("Gazebo/Blue");


  xTube->attachObject(tubeObj1);
  yTube->attachObject(tubeObj2);
  zTube->attachObject(tubeObj3);

  xBox->attachObject(boxObj1);
  xBox->setInheritOrientation(false);
  xBox->setScale(0.1, 0.1, 0.1);
  xBox->setPosition(1.5, 0, 0);

  yBox->attachObject(boxObj2);
  yBox->setInheritOrientation(false);
  yBox->setScale(0.1, 0.1, 0.1);
  yBox->setPosition(0, 1.5, 0);

  zBox->attachObject(boxObj3);
  zBox->setInheritOrientation(false);
  zBox->setScale(0.1, 0.1, 0.1);
  zBox->setPosition(0, 0, 1.5);

}

////////////////////////////////////////////////////////////////////////////////
// Attach to a scene node
void SelectionObj::Attach( Ogre::SceneNode *sceneNode )
{
  if (sceneNode)
  {
    if (this->node->getParent())
      this->node->getParent()->removeChild(this->node);

    sceneNode->addChild( this->node );
    Ogre::Vector3 scale = sceneNode->getScale();
    this->node->setScale(scale.x, scale.y , scale.z);

    this->node->setVisible(true);
  }
  else
  {
    if (this->node->getParent())
      this->node->getParent()->removeChild(this->node);

    this->node->setVisible(false);
  }

}
