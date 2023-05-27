#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "pathfindingwidget.h"
#include "tracedialog.h"

TraceDialog *traceDialog=NULL;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->widget->mainWnd=this;
    ui->widget->wid=1000;
    ui->widget->hei=550;
    ui->widget->timeInterval=40;

    connect(ui->pushButton_generateScene,SIGNAL(clicked()),ui->widget,SLOT(fgenerate()));
    connect(ui->pushButton_updatePoints,SIGNAL(clicked()),ui->widget,SLOT(fgenerateVisiblePoints()));
    connect(ui->pushButton_custom,SIGNAL(clicked()),ui->widget,SLOT(fcustom()));

    connect(ui->pushButton_spawn,SIGNAL(clicked()),ui->widget,SLOT(fspawn()));
    connect(ui->pushButton_updateEntities,SIGNAL(clicked()),ui->widget,SLOT(fupdate()));
    connect(ui->pushButton_trace,SIGNAL(clicked()),ui->widget,SLOT(ftracebtn()));


    connect(ui->narrowDisplay_check,SIGNAL(clicked(bool)),ui->widget,SLOT(fnarrowDisplay()));
    connect(ui->multiAvoid_check,SIGNAL(clicked(bool)),ui->widget,SLOT(fmultiAvoid()));

    ui->widget->fload();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::displayTraceDialog(){
    if(traceDialog==NULL){
        traceDialog=new TraceDialog(&traceDialog,this);
        traceDialog->setModal(false);
        traceDialog->show();
    }
}
