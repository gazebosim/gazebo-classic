#include <QtGui>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"
#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"

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
  connect(this->fileTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)) );

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  this->addButton = new QPushButton(tr("Apply"));
  connect(this->addButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  this->cancelButton = new QPushButton(tr("Cancel"));
  connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  buttonLayout->addWidget(this->addButton);
  buttonLayout->addWidget(this->cancelButton);


  mainLayout->addWidget(this->fileTreeWidget);
  mainLayout->addLayout(buttonLayout);
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
  this->node->Init();

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->visualPub = this->node->Advertise<msgs::Visual>("~/visual");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");
}

InsertModelWidget::~InsertModelWidget()
{
}

//void InsertModelWidget::OnModelSelection()
void InsertModelWidget::OnModelSelection(QTreeWidgetItem *item, int column)
{
  gzdbg << "OnModelSelection\n";
  QTreeWidgetItem* selected = this->fileTreeWidget->currentItem();
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  scene->RemoveVisual( this->modelVisual );
  this->visuals.clear();

  if (selected)
  {
    std::string path, filename;

    if (selected->parent())
      path = selected->parent()->text(0).toStdString() + "/";

    filename = selected->text(0).toStdString();
    this->selectedModel = path+filename;

    if (filename.find(".model") == std::string::npos)
      return;

    sdf::SDFPtr modelSDF(new sdf::SDF);
    sdf::initFile( "/sdf/gazebo.sdf", modelSDF );
    sdf::readFile( this->selectedModel, modelSDF);

    // Load the world file
    std::string modelName;
    math::Pose modelPose, linkPose, visualPose;

    sdf::ElementPtr modelElem = modelSDF->root->GetElement("model");
    if (modelElem->HasElement("origin"))
      modelPose = modelElem->GetElement("origin")->GetValuePose("pose");

    modelName = modelElem->GetValueString("name");
    this->modelVisual.reset(new rendering::Visual(modelName, scene));
    this->modelVisual->Load();
    this->modelVisual->SetPose(modelPose);

    scene->AddVisual(modelVisual);

    this->visuals.push_back(modelVisual);

    sdf::ElementPtr linkElem = modelElem->GetElement("link");
    while (linkElem)
    {
      std::string linkName = linkElem->GetValueString("name");
      if (linkElem->HasElement("origin"))
        linkPose = linkElem->GetElement("origin")->GetValuePose("pose");

      gzdbg << "LinkName[" << linkName << "]\n";

      rendering::VisualPtr linkVisual( new rendering::Visual(modelName+"::"+linkName, modelVisual) );
      linkVisual->Load();
      linkVisual->SetPose( linkPose );
      this->visuals.push_back(linkVisual);

      int visualIndex = 0;
      sdf::ElementPtr visualElem = linkElem->GetElement("visual");
      while (visualElem)
      {
        if (visualElem->HasElement("origin"))
          visualPose = visualElem->GetElement("origin")->GetValuePose("pose");

        std::ostringstream visualName;
        visualName << modelName << "::" << linkName << "::Visual_" 
                   << visualIndex++;
        rendering::VisualPtr visVisual( new rendering::Visual(visualName.str(), linkVisual) );
        visVisual->Load(visualElem);
        visVisual->SetTransparency(0.5);
        this->visuals.push_back(visVisual);


        visualElem = linkElem->GetNextElement("visual", visualElem);
      }
 
      linkElem = modelElem->GetNextElement("link", linkElem);
    }

    msgs::Selection selectMsg;
    msgs::Init(selectMsg, modelName);
    selectMsg.set_selected(true);
    this->selectionPub->Publish(selectMsg);
  }
}

void InsertModelWidget::OnApply()
{
  msgs::Factory msg;
  msgs::Init(msg, "new_model");
  msg.set_filename(this->selectedModel);

  gzdbg << "Pose[" << this->modelVisual->GetWorldPose()  << "]\n";
  msg.mutable_pose()->CopyFrom( msgs::Convert(this->modelVisual->GetWorldPose()) );
  this->factoryPub->Publish(msg);

  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  scene->RemoveVisual( this->modelVisual );
  this->visuals.clear();
}

void InsertModelWidget::OnCancel()
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  scene->RemoveVisual( this->modelVisual );
  this->visuals.clear();
}
