#ifndef TRACEDIALOG_H
#define TRACEDIALOG_H

#include <QDialog>

namespace Ui {
class TraceDialog;
}

class TraceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TraceDialog(TraceDialog **ref0,QWidget *parent = 0);
    ~TraceDialog();
    void traceData(QString str);

    TraceDialog **ref;
private:
    Ui::TraceDialog *ui;
};

#endif // TRACEDIALOG_H
