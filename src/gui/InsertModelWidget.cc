#include <QtGui>
#include <boost/filesystem.hpp>

#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "gui/InsertModelWidget.hh"

using namespace gazebo;
using namespace gui;

InsertModelWidget::InsertModelWidget( QWidget *parent )
  : QWidget( parent )
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->fileTreeWidget = new QTreeWidget();
  this->fileTreeWidget->setColumnCount(1);

  mainLayout->addWidget(this->fileTreeWidget);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  QList<QTreeWidgetItem*> items;
  std::list<std::string> gazeboPaths = common::SystemPaths::Instance()->GetGazeboPaths();

  for (std::list<std::string>::iterator iter = gazeboPaths.begin(); 
       iter != gazeboPaths.end(); iter++)
  {
    QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, QStringList(QString("%1").arg( QString::fromStdString(*iter)) ));

    this->fileTreeWidget->addTopLevelItem(topItem);

    boost::filesystem::path dir((*iter) + common::SystemPaths::Instance()->GetModelPathExtension());
    boost::filesystem::directory_iterator endIter;
    std::list<boost::filesystem::path> resultSet;

    if ( boost::filesystem::exists(dir) && 
         boost::filesystem::is_directory(dir) )
    {
      for( boost::filesystem::directory_iterator dirIter(dir); 
           dirIter != endIter; ++dirIter)
      {
        if ( boost::filesystem::is_regular_file(dirIter->status()) )
        {
          QTreeWidgetItem *childItem = new QTreeWidgetItem( topItem, 
              QStringList(QString("%1").arg( 
                  QString::fromStdString( dirIter->path().root_name() )) ));
          this->fileTreeWidget->addTopLevelItem(childItem);
          gzdbg << dirIter->path().root_name() << "\n";
        }
      }
    }
  }


}

InsertModelWidget::~InsertModelWidget()
{
}
