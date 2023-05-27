#include "tracedialog.h"
#include "ui_tracedialog.h"

TraceDialog::TraceDialog(TraceDialog **ref0,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TraceDialog)
{
    ref=ref0;

    ui->setupUi(this);



    setAttribute (Qt::WA_DeleteOnClose);

    Qt::WindowFlags flags=Qt::Dialog;
    flags |=Qt::WindowMinimizeButtonHint;
    flags |=Qt::WindowMaximizeButtonHint;
    flags |=Qt::WindowCloseButtonHint;
    setWindowFlags(flags);
}

TraceDialog::~TraceDialog()
{
    delete ui;
    if(ref!=NULL){
        *ref=NULL;
    }
}

void TraceDialog::traceData(QString str){
    ui->plainTextEdit->appendPlainText(str);
}
