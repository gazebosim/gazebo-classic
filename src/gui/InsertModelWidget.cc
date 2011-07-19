#include <QtGui>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/InsertModelWidget.hh"

using namespace gazebo;
using namespace gui;

InsertModelWidget::InsertModelWidget( QWidget *parent )
  : QWidget( parent )
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->fileTreeWidget = new QTreeWidget();
  this->fileTreeWidget->setColumnCount(1);
  connect(this->fileTreeWidget, SIGNAL(itemSelectionChanged()),
          this, SLOT(OnModelSelection()) );



  mainLayout->addWidget(this->fileTreeWidget);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  QList<QTreeWidgetItem*> items;
  std::list<std::string> gazeboPaths = common::SystemPaths::GetGazeboPaths();

  // Iterate over all the gazebo paths
  for (std::list<std::string>::iterator iter = gazeboPaths.begin(); 
       iter != gazeboPaths.end(); iter++)
  {
    // This is the full model path
    std::string path = (*iter) + common::SystemPaths::GetModelPathExtension();

    // Create a top-level tree item for the path
    QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, QStringList(QString("%1").arg( QString::fromStdString(path)) ));
    this->fileTreeWidget->addTopLevelItem(topItem);

    boost::filesystem::path dir(path);
    boost::filesystem::directory_iterator endIter;
    std::list<boost::filesystem::path> resultSet;

    if ( boost::filesystem::exists(dir) && 
         boost::filesystem::is_directory(dir) )
    {
      // Iterate over all the model in the current gazebo path
      for( boost::filesystem::directory_iterator dirIter(dir); 
           dirIter != endIter; ++dirIter)
      {
        if ( boost::filesystem::is_regular_file(dirIter->status()) )
        {
          // Add a child item for the model
          QTreeWidgetItem *childItem = new QTreeWidgetItem( topItem, 
              QStringList(QString("%1").arg( 
                  QString::fromStdString( dirIter->path().leaf() )) ));
          this->fileTreeWidget->addTopLevelItem(childItem);
        }
      }
    }
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->visualPub = this->node->Advertise<msgs::Visual>("~/visual");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");
}

InsertModelWidget::~InsertModelWidget()
{
}

void InsertModelWidget::OnModelSelection()
{
  /*QTreeWidgetItem* selected = this->fileTreeWidget->selectedItems().at(0);
  if (selected)
  {
    std::string path = selected->parent()->text(0).toStdString() + "/";
    std::string filename = selected->text(0).toStdString();

    // Load the world file
    gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();
    try
    {
      xmlFile->Load(path + filename);
    }
    catch (common::Exception e)
    {
      gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
    }

    std::string modelName;

    common::XMLConfigNode *modelNode = xmlFile->GetRootNode();
    if (modelNode)
    {
      int count = 0;
      modelName = modelNode->GetString("name","",1);

      // Create the model-level visual msg
      msgs::Visual visMsg;
      msgs::Init(visMsg, modelName);
      this->visualPub->Publish(visMsg);

      common::XMLConfigNode *linkNode = modelNode->GetChild("link");
      while (linkNode)
      {
        std::string linkName = linkNode->GetString("name","",1);
        common::XMLConfigNode *originNode = linkNode->GetChild("origin");

        math::Vector3 pos;
        math::Quaternion rot;

        if (originNode)
        {
          pos = originNode->GetVector3("xyz", math::Vector3());
          rot = originNode->GetRotation("rpy",math::Quaternion());
        }

        msgs::Init(visMsg, modelName+"::"+linkName);
        msgs::Set(visMsg.mutable_pose()->mutable_position(), pos);
        msgs::Set(visMsg.mutable_pose()->mutable_orientation(), rot);

        visMsg.set_parent_id(modelName);
        this->visualPub->Publish(visMsg);

        common::XMLConfigNode *visualNode = linkNode->GetChild("visual");
        while (visualNode)
        {
          std::string name = boost::lexical_cast<std::string>(count++);

          visMsg = msgs::VisualFromXML(visualNode);
          msgs::Init(visMsg, name);
          visMsg.set_parent_id(modelName + "::" + linkName);

          this->visualPub->Publish(visMsg);

          visualNode = visualNode->GetNext("visual");
        }
        linkNode = linkNode->GetNext("link");
      }
    }

    msgs::Selection selectMsg;
    msgs::Init(selectMsg, modelName);
    selectMsg.set_selected(true);
    this->selectionPub->Publish(selectMsg);

    //msgs::Factory msg;
    //msgs::Init(msg, "new");
    //msg.set_filename(path+filename);

    //this->factoryPub->Publish(msg);
  }
*/
}
