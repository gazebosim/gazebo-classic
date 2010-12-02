#include <Ogre.h>
#include <boost/lexical_cast.hpp>
#include "Scene.hh"
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
    OgreAdaptor::Instance()->GetScene(0)->GetManager()->getRootSceneNode()->removeAndDestroyChild( this->node->getName() );
}

////////////////////////////////////////////////////////////////////////////////
// Load
void SelectionObj::Load()
{
  Ogre::SceneNode *pnode = OgreAdaptor::Instance()->GetScene(0)->GetManager()->getRootSceneNode();
  this->node = pnode->createChildSceneNode("selection_node");

  Ogre::SceneNode *rotNode = this->node->createChildSceneNode("rot_node");
  Ogre::SceneNode *transNode = this->node->createChildSceneNode("trans_node");
  transNode->setInheritOrientation(false);
  
  if (!this->node->getCreator()->hasEntity("selection_tube"))
    OgreCreator::InsertMesh(MeshManager::Instance()->GetMesh("selection_tube"));
  if (!this->node->getCreator()->hasEntity("unit_box_U1V1"))
    OgreCreator::InsertMesh(MeshManager::Instance()->GetMesh("unit_box_U1V1"));


  Ogre::SceneNode *xTube, *yTube, *zTube, *xBox[2], *yBox[2], *zBox[2];

  xTube = rotNode->createChildSceneNode("selection_nodeX");
  xTube->yaw(Ogre::Radian(M_PI/2.0));

  yTube = rotNode->createChildSceneNode("selection_nodeY");
  yTube->pitch(Ogre::Radian(M_PI/2.0));

  zTube = rotNode->createChildSceneNode("selection_nodeZ");

  for (int i=0; i < 2; i++)
  {
    xBox[i] = transNode->createChildSceneNode("selection_transX" + boost::lexical_cast<std::string>(i));
    xBox[i]->setInheritScale(true);

    yBox[i] = transNode->createChildSceneNode("selection_transY" + boost::lexical_cast<std::string>(i));
    yBox[i]->setInheritScale(true);

    zBox[i] = transNode->createChildSceneNode("selection_transZ" + boost::lexical_cast<std::string>(i));
    zBox[i]->setInheritScale(true);
  }


  Ogre::MovableObject *tubeObj1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ1", "selection_tube"));
  tubeObj1->setCastShadows(false);
  tubeObj1->setUserAny( Ogre::Any(std::string("rotx")) );
  ((Ogre::Entity*)tubeObj1)->setMaterialName("Gazebo/RedTransparent");

  Ogre::MovableObject *tubeObj2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ2", "selection_tube"));
  tubeObj2->setCastShadows(false);
  tubeObj2->setUserAny( Ogre::Any(std::string("roty")) );
  ((Ogre::Entity*)tubeObj2)->setMaterialName("Gazebo/GreenTransparent");

  Ogre::MovableObject *tubeObj3 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJ3", "selection_tube"));
  tubeObj3->setCastShadows(false);
  tubeObj3->setUserAny( Ogre::Any(std::string("rotz")) );
  ((Ogre::Entity*)tubeObj3)->setMaterialName("Gazebo/BlueTransparent");

  Ogre::MovableObject *boxObjX1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJX1", "unit_box_U1V1"));
  boxObjX1->setCastShadows(false);
  boxObjX1->setUserAny( Ogre::Any(std::string("transx")) );
  ((Ogre::Entity*)boxObjX1)->setMaterialName("Gazebo/RedTransparent");

  Ogre::MovableObject *boxObjX2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJX2", "unit_box_U1V1"));
  boxObjX2->setCastShadows(false);
  boxObjX2->setUserAny( Ogre::Any(std::string("transx")) );
  ((Ogre::Entity*)boxObjX2)->setMaterialName("Gazebo/RedTransparent");


  Ogre::MovableObject *boxObjY1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJY1", "unit_box_U1V1"));
  boxObjY1->setCastShadows(false);
  boxObjY1->setUserAny( Ogre::Any(std::string("transy")) );
  ((Ogre::Entity*)boxObjY1)->setMaterialName("Gazebo/GreenTransparent");

  Ogre::MovableObject *boxObjY2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJY2", "unit_box_U1V1"));
  boxObjY2->setCastShadows(false);
  boxObjY2->setUserAny( Ogre::Any(std::string("transy")) );
  ((Ogre::Entity*)boxObjY2)->setMaterialName("Gazebo/GreenTransparent");


  Ogre::MovableObject *boxObjZ1 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJZ1", "unit_box_U1V1"));
  boxObjZ1->setCastShadows(false);
  boxObjZ1->setUserAny( Ogre::Any(std::string("transz")) );
  ((Ogre::Entity*)boxObjZ1)->setMaterialName("Gazebo/BlueTransparent");

  Ogre::MovableObject *boxObjZ2 = (Ogre::MovableObject*)(this->node->getCreator()->createEntity("SELECTION_OBJZ2", "unit_box_U1V1"));
  boxObjZ2->setCastShadows(false);
  boxObjZ2->setUserAny( Ogre::Any(std::string("transz")) );
  ((Ogre::Entity*)boxObjZ2)->setMaterialName("Gazebo/BlueTransparent");


  xTube->attachObject(tubeObj1);
  yTube->attachObject(tubeObj2);
  zTube->attachObject(tubeObj3);

  xBox[0]->attachObject(boxObjX1);
  xBox[0]->setInheritOrientation(false);
  xBox[0]->setScale(0.2, 0.2, 0.2);
  xBox[0]->setPosition(1.5, 0, 0);

  xBox[1]->attachObject(boxObjX2);
  xBox[1]->setInheritOrientation(false);
  xBox[1]->setScale(0.2, 0.2, 0.2);
  xBox[1]->setPosition(-1.5, 0, 0);

  yBox[0]->attachObject(boxObjY1);
  yBox[0]->setInheritOrientation(false);
  yBox[0]->setScale(0.2, 0.2, 0.2);
  yBox[0]->setPosition(0, 1.5, 0);

  yBox[1]->attachObject(boxObjY2);
  yBox[1]->setInheritOrientation(false);
  yBox[1]->setScale(0.2, 0.2, 0.2);
  yBox[1]->setPosition(0, -1.5, 0);


  zBox[0]->attachObject(boxObjZ1);
  zBox[0]->setInheritOrientation(false);
  zBox[0]->setScale(0.2, 0.2, 0.2);
  zBox[0]->setPosition(0, 0, 1.5);

  zBox[1]->attachObject(boxObjZ2);
  zBox[1]->setInheritOrientation(false);
  zBox[1]->setScale(0.2, 0.2, 0.2);
  zBox[1]->setPosition(0, 0, -1.5);

}

////////////////////////////////////////////////////////////////////////////////
// Attach to a scene node
void SelectionObj::Attach( Ogre::SceneNode *sceneNode )
{
  if (sceneNode)
  {
    if (this->node->getParent())
      this->node->getParent()->removeChild(this->node);

    Ogre::AxisAlignedBox box;
    OgreCreator::GetSceneNodeBounds(sceneNode, box);
    Ogre::Vector3 boxmin = box.getMinimum();
    Ogre::Vector3 boxmax = box.getMaximum();

    Vector3 scale( boxmax.x - boxmin.x, boxmax.y - boxmin.y, 
                   boxmax.z - boxmin.z );

    double max = std::max(scale.x, scale.y);
    max = std::max(max, scale.z);
    sceneNode->addChild( this->node );
    this->node->setScale(max, max, max);

    this->node->setVisible(true);
  }
  else
  {
    if (this->node->getParent())
      this->node->getParent()->removeChild(this->node);

    this->node->setVisible(false);
  }

}
