#include "pathfindingwidget.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "tracedialog.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <QPainter>
#include <QMouseEvent>
#include <qDebug>

#define PI 3.14159265358979

using namespace std;

extern TraceDialog *traceDialog;


PathFindingWidget::PathFindingWidget(QWidget *parent) : QWidget(parent)
{


    srand((unsigned)time(NULL));
    mainWnd=NULL;

    wid=800;
    hei=600;
    columnMode=false;
    wallGroupColumnCount=0;
    groupWidth=500;
    groupHeight=500;
    groupOffsetX=0;
    groupOffsetY=0;

    hasDest=false;

    timeInterval=50;
    multiAvoid=true;
    state=0;
    timeCount=0;
    hasTraced=true;
    willNarrow=true;

    drawingObstales=false;
    polygon0.clear();
    allRectObstales=false;
    this->setAutoFillBackground(true);
    //QPalette palette;
    //palette.setColor(QPalette::Background, QColor(127,127,127));
    //this->setPalette(palette);
}

PathFindingWidget::~PathFindingWidget()
{
    mainWnd=NULL;
}

void PathFindingWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    displayToPainter(&painter);
}

void PathFindingWidget::mousePressEvent(QMouseEvent *event)
{
    int i=0,length=0;
    if(event->button()==Qt::LeftButton){
        if(drawingObstales){
            if(polygon0.size()==0){
                vector<double> polygoni0;
                polygon0.push_back(polygoni0);
            }
            if(polygon0.size()==0){
                return;
            }
            vector<double> &polygon0t=polygon0[polygon0.size()-1];
            double polygonxt=event->x();
            double polygonyt=event->y();
            if(polygon0t.size()<2 || polygonxt!=polygon0t[polygon0.size()-2] || polygonyt!=polygon0t[polygon0.size()-1]){
                polygon0t.push_back(polygonxt);
                polygon0t.push_back(polygonyt);
            }else if(polygon0t.size()>=4 && polygonxt==polygon0t[0] && polygonyt==polygon0t[1]){
                vector<double> polygonit;
                polygon0.push_back(polygonit);
            }
            update();
            return;
        }
        hasDest=true;
        destX=event->x();
        destY=event->y();
        i=0;
        length=(int)(circleArray.size());
        pathArray.resize(length);
        for(i=0;i<length;i++){
            findPath(circleArray[i][0],circleArray[i][1],destX,destY,pathArray[i]);
            circleArray[i][6]=(!pathArray.empty())?0:-1;
        }
        state=0;
        hasTraced=false;
        timeCount=0;
        update();
    }else if(event->button()==Qt::RightButton){
        pathArray.clear();
        hasDest=false;
        destX=0;
        destY=0;
        i=0;
        if(!circleArray.empty()){
            length=(int)(circleArray.size());
            for(i=0;i<length;i++){
                circleArray[i][6]=-1;
            }
        }
        state=0;
        hasTraced=true;
        timeCount=0;
        update();
    }
}


void PathFindingWidget::fload(){
    willNarrow=mainWnd->ui->narrowDisplay_check->isChecked();
    willNarrow=willNarrow && atof(mainWnd->ui->thickness->text().toStdString().c_str())>0;
    //willNarrow=willNarrow && atof(mainWnd->ui->entityCount->text().toStdString().c_str())>0;
    drawingObstales=false;
    fgenerate();
    startTimer(timeInterval);
}

void PathFindingWidget::fgenerate(){
    if(mainWnd==NULL){
       return;
    }

    int count=atoi(mainWnd->ui->obstacleCount->text().toStdString().c_str());
    double thickness=atof(mainWnd->ui->thickness->text().toStdString().c_str());
    double margin=20+thickness;
    willNarrow=mainWnd->ui->narrowDisplay_check->isChecked();
    willNarrow=willNarrow && atof(mainWnd->ui->thickness->text().toStdString().c_str())>0;
    //willNarrow=willNarrow && atof(mainWnd->ui->entityCount->text().toStdString().c_str())>0;
    if(drawingObstales){
        /*if(polygon0.size()>0 && polygon0[polygon0.size()-1].size()>=6){
            vector<double> polygonit;
            polygon0.push_back(polygonit);
        }*/
        mainWnd->ui->pushButton_custom->setText("自定");
        mainWnd->ui->pushButton_generateScene->setText("生成");
        mainWnd->ui->pushButton_updatePoints->setText("更新");
        drawingObstales=false;
        double r=atof(mainWnd->ui->radius->text().toStdString().c_str());
        createWallArrayFromPolygon0(r);
    }else{
        polygon0.clear();
        createRandomWallArray(count,margin,margin,wid-margin,hei-margin,thickness,-2);
    }


    /*if(groupWidth0>0 && groupHeight0>0){
        groupWidth=groupWidth0;
        groupHeight=groupHeight0;
        updateGroup();
    }*/

    fgenerateVisiblePoints();

    pathArray.clear();

    hasDest=false;
    destX=0,destY=0;
    fspawn();

    multiAvoid=mainWnd->ui->multiAvoid_check->isChecked();

    update();
}

void PathFindingWidget::fgenerateVisiblePoints(){
    if(drawingObstales){
        polygon0.clear();
        mainWnd->ui->pushButton_custom->setText("自定");
        mainWnd->ui->pushButton_generateScene->setText("生成");
        mainWnd->ui->pushButton_updatePoints->setText("更新");
        drawingObstales=false;
        fgenerate();
        return;
    }
    columnMode=mainWnd->ui->column_check->isChecked();
    double wayPointArrayOffset=atof(mainWnd->ui->offset->text().toStdString().c_str());
    double thickness=atof(mainWnd->ui->thickness->text().toStdString().c_str());

    if(wayPointArrayOffset>0.00001){
        createWayPointArray(wayPointArrayOffset);
    }else{
        wayPointArray.clear();
    }
    connectionGraph.clear();
    connectionDistance.clear();

    if(columnMode){
        createWayPointColumn(true);
    }else if(thickness>0){
        clearInnerWayPoints();
        //clearInnerColumnWayPoints();
    }


    pathArray.clear();

    hasDest=false;
    destX=0,destY=0;
    update();
}

void PathFindingWidget::fcustom(){
    if(drawingObstales){
        if(polygon0.size()>0 && polygon0[polygon0.size()-1].size()>=6){
            vector<double> polygonit;
            polygon0.push_back(polygonit);
        }
        update();
        return;
    }
    mainWnd->ui->pushButton_custom->setText("闭合");
    mainWnd->ui->pushButton_generateScene->setText("确定");
    mainWnd->ui->pushButton_updatePoints->setText("取消");
    drawingObstales=true;
    polygon0.clear();
    update();
}

void PathFindingWidget::fspawn(){
    int count=atof(mainWnd->ui->entityCount->text().toStdString().c_str());
    double r=atof(mainWnd->ui->radius->text().toStdString().c_str());
    dist=atof(mainWnd->ui->dist->text().toStdString().c_str());
    interv=atof(mainWnd->ui->interv->text().toStdString().c_str());
    wayPointArriveR=atof(mainWnd->ui->wayPointRadius->text().toStdString().c_str());
    if(interv<r){
        interv=r;
        //mainWnd->ui->interv->setText(interv);
    }
    velocity=atof(mainWnd->ui->velocity->text().toStdString().c_str());
    omega=atof(mainWnd->ui->omega->text().toStdString().c_str());

    circleArray.resize(count);
    int i=0,j=0;
    bool willAdd=true;
    double dx,dy,rt;
    while(i<count){
        willAdd=true;
        circleArray[i].resize(7);
        circleArray[i][0]=random()*(wid-2*r)+r;
        circleArray[i][1]=random()*(hei-2*r)+r;
        circleArray[i][2]=r;
        circleArray[i][3]=-1;
        circleArray[i][4]=-1;
        circleArray[i][5]=360*random()-180;
        circleArray[i][6]=-1;
        if(inClosedPolygon(circleArray[i][0],circleArray[i][1])){
            willAdd=false;
            continue;
        }
        for(j=0;j<i;j++){
            dx=circleArray[i][0]-circleArray[j][0];
            dy=circleArray[i][1]-circleArray[j][1];
            rt=circleArray[i][2]+circleArray[j][2];
            if(dx*dx+dy*dy<=rt*rt){
                willAdd=false;
                break;
            }
        }
        if(willAdd){
            i++;
        }
    }
    hasDest=false;
    destX=0,destY=0;

    hasTraced=true;
    pathArray.clear();
    update();
}

void PathFindingWidget::fupdate(){
    dist=atof(mainWnd->ui->dist->text().toStdString().c_str());
    interv=atof(mainWnd->ui->interv->text().toStdString().c_str());
    velocity=atof(mainWnd->ui->velocity->text().toStdString().c_str());
    omega=atof(mainWnd->ui->omega->text().toStdString().c_str());
    wayPointArriveR=atof(mainWnd->ui->wayPointRadius->text().toStdString().c_str());
    state=0;
    hasTraced=false;
}

void PathFindingWidget::fcolumn(){

}

void PathFindingWidget::fnarrowDisplay(){
    willNarrow=mainWnd->ui->narrowDisplay_check->isChecked();
    willNarrow=willNarrow && atof(mainWnd->ui->thickness->text().toStdString().c_str())>0;
    //willNarrow=willNarrow && atof(mainWnd->ui->entityCount->text().toStdString().c_str())>0;
    update();
}

void PathFindingWidget::fmultiAvoid(){
    multiAvoid=mainWnd->ui->multiAvoid_check->isChecked();
    fupdate();
}

double PathFindingWidget::random(){
    //生成[0,1)的随机浮点数
    return rand()/(double)(RAND_MAX);
}

void PathFindingWidget::displayToPainter(QPainter* painter){
    //显示障碍物，可见点和单位到画布painter上
    int i=0,j=0;
    double lxprev=0,lyprev=0;

    painter->fillRect(0,0,wid,hei,QColor(255,255,255));
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(QColor(0,0,0));
    painter->setBrush(Qt::NoBrush);

    if(drawingObstales || polygon0.size()>0 && willNarrow){
        for(i=0;i<(int)(polygon0.size());i++){
            if(polygon0[i].size()<2){
                continue;
            }
            for(j=2;j+1<(int)(polygon0[i].size());j+=2){
                painter->drawLine(polygon0[i][j-2],polygon0[i][j-1],polygon0[i][j],polygon0[i][j+1]);
            }
            if(!drawingObstales || i<(int)(polygon0.size())-1){
                painter->drawLine(polygon0[i][j-2],polygon0[i][j-1],polygon0[i][0],polygon0[i][1]);
            }
        }
        if(drawingObstales){
            return;
        }
    }else if(willNarrow){
        vector<double> wideArray;
        vector< vector<double> > narrowArray;
        wideArray.resize(8);
        double rt=circleArray[0][2];
        painter->setPen(QColor(0,0,0));
        for(i=0;i<(int)(wallArray.size());i+=16){
            wideArray[0]=wallArray[i];
            wideArray[1]=wallArray[i+1];
            wideArray[2]=wallArray[i+2];
            wideArray[3]=wallArray[i+3];
            wideArray[4]=wallArray[i+6];
            wideArray[5]=wallArray[i+7];
            wideArray[6]=wallArray[i+10];
            wideArray[7]=wallArray[i+11];
            widenParallelogram(wideArray,-rt,narrowArray);
            painter->drawLine(narrowArray[0][0],narrowArray[0][1],narrowArray[1][0],narrowArray[1][1]);
            painter->drawLine(narrowArray[1][0],narrowArray[1][1],narrowArray[2][0],narrowArray[2][1]);
            painter->drawLine(narrowArray[2][0],narrowArray[2][1],narrowArray[3][0],narrowArray[3][1]);
            painter->drawLine(narrowArray[3][0],narrowArray[3][1],narrowArray[0][0],narrowArray[0][1]);
        }
    }else{
        painter->setPen((!willNarrow)?QColor(0,0,0):QColor(0xcc,0xcc,0xcc));


        for(i=0;i+3<(int)(wallArray.size());i+=4){
            painter->drawLine(wallArray[i],wallArray[i+1],wallArray[i+2],wallArray[i+3]);
        }
    }


    if(!columnMode){
        for(i=0;i<(int)(wayPointArray.size());i+=2){
            painter->drawEllipse(QPointF(wayPointArray[i],wayPointArray[i+1]),3,3);
        }
    }else{
        if(!willNarrow){
            painter->setPen(QColor(0x7f,0xff,0xff));
            for(i=0;i<(int)(columnLine.size());i++){
                painter->drawLine(columnLine[i][0],columnLine[i][1],columnLine[i][2],columnLine[i][3]);
            }
        }
        painter->setPen(QColor(0,0,0));
        //destShape.strokeStyle = "#000";
        for(i=0;i<(int)(wayPointArray.size());i+=2){
            painter->drawEllipse(QPointF(wayPointArray[i],wayPointArray[i+1]),3,3);
        }
    }
    //destShape.strokeStyle = "#000";
    //destShape.beginPath();
    //destShape.arc(unit.x,unit.y,7.5,0,2*Math.PI,false);
    //destShape.closePath();
    //destShape.stroke();
    if(pathArray.size()>0){

        painter->setPen(QColor(0xe0,0xe0,0xe0));
        for(j=0;j<(int)(pathArray.size());j++){
            if(pathArray.size()<4){
                continue;
            }

            vector<double> &pak=pathArray[j];
            lxprev=circleArray[j][0];
            lyprev=circleArray[j][1];
            for(i=2*circleArray[j][6]+2;i<(int)(pak.size());i+=2){
                painter->drawLine(lxprev,lyprev,pak[i],pak[i+1]);
                lxprev=pak[i];
                lyprev=pak[i+1];
            }
        }
    }

    painter->setPen(QColor(0,0,0));
    int count=(int)(circleArray.size());
    double arc0=PI-0.5;
    double rad0=0;
    QBrush brushBlack(QColor(0,0,0));
    QPainterPath painterPath;
    for(i=0;i<count;i++){
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(QPointF(circleArray[i][0],circleArray[i][1]),circleArray[i][2],circleArray[i][2]);

        painter->setBrush(brushBlack);
        rad0=circleArray[i][5]*PI/180;
        painterPath=QPainterPath();
        painterPath.moveTo(circleArray[i][0]+circleArray[i][2]*cos(rad0-arc0),circleArray[i][1]+circleArray[i][2]*sin(rad0-arc0));
        painterPath.lineTo(circleArray[i][0]+circleArray[i][2]*cos(rad0),circleArray[i][1]+circleArray[i][2]*sin(rad0));
        painterPath.lineTo(circleArray[i][0]+circleArray[i][2]*cos(rad0+arc0),circleArray[i][1]+circleArray[i][2]*sin(rad0+arc0));
        painterPath.closeSubpath();
        painter->drawPath(painterPath);
    }
    if(hasDest){
        painter->setBrush(brushBlack);
        painter->drawEllipse(destX-1,destY,3,3);
    }
}

void PathFindingWidget::createWayPointArray(double offset,std::vector<double> *outWayPointArray){
    //从障碍数组附近，创建路径点数组，并在闭合三角形和四边形存在时建立多边形数组，参数是端点处延长系数
    if(wallArray.empty() || (int)(wallArray.size())<4){
        //qDebug("创建路径点数组时，障碍数组不能为空");
        if(outWayPointArray==NULL){
            wayPointArray.clear();
        }
        return;
    }

    int i=0;
    int leng=0;(allRectObstales || polygonArray.size()==0)?(((int)(wallArray.size())>>2)<<2):0;
    vector<double> ddArray;
    ddArray.resize(4);
    vector<double> &wayPointArrayt=outWayPointArray!=NULL?*outWayPointArray:wayPointArray;
    if((int)(wayPointArrayt.size())!=leng){
        wayPointArrayt.resize(leng);
    }


    if(polygonArray.size()==0){
        leng=((int)(wallArray.size())>>2)<<2;
        if((int)(wayPointArrayt.size())!=leng){
            wayPointArrayt.resize(leng);
        }
        for(i=0;i<leng;i+=4){
            if(offset==0){
                wayPointArrayt[i]=wallArray[i];
                wayPointArrayt[i+1]=wallArray[i+1];
                wayPointArrayt[i+2]=wallArray[i+2];
                wayPointArrayt[i+3]=wallArray[i+3];
                continue;
            }
            ddcyc(wallArray[i],wallArray[i+1],wallArray[i+2],wallArray[i+3],offset,ddArray);
            wayPointArrayt[i]=ddArray[0];
            wayPointArrayt[i+1]=ddArray[1];
            wayPointArrayt[i+2]=ddArray[2];
            wayPointArrayt[i+3]=ddArray[3];
        }
    }else{
        int j=0,k=(int)(polygonArray.size()),l=0,m=0;
        for(i=0;i+1<k;i+=2){
            leng+=2*polygonArray[i+1];
        }
        if((int)(wayPointArrayt.size())!=leng){
            wayPointArrayt.resize(leng);
        }
        int wallIdmin=0,wallIdmax=0;
        vector<double> lengarr,ratearr;
        double dx,dy,dx2,dy2,d2;
        double sinarr,cosarr;

        for(i=0;i+1<k;i+=2){
            wallIdmin=polygonArray[i];
            wallIdmax=wallIdmin+4*polygonArray[i+1]-1;
            if(wallIdmax-wallIdmin<11){
                continue;
            }
            if(wallIdmin<0 || wallIdmax>=(int)(wallArray.size())){
                continue;
            }

            if(offset==0){
                for(j=wallIdmin;j+3<=wallIdmax;j+=4){
                    if(l+1<leng){
                        wayPointArrayt[l]=wallArray[j];
                        wayPointArrayt[l+1]=wallArray[j+1];
                        l+=2;
                    }
                }
                continue;
            }
            lengarr.resize(polygonArray[i+1]);
            ratearr.resize(polygonArray[i+1]);
            for(j=wallIdmin;j+3<=wallIdmax;j+=4){
                dx=wallArray[j+2]-wallArray[j];
                dy=wallArray[j+3]-wallArray[j+1];
                d2=dx*dx+dy*dy;
                lengarr[(j-wallIdmin)>>2]=sqrt(d2);
            }
            for(j=wallIdmin;j+3<=wallIdmax;j+=4){
                if(j-4<wallIdmin){
                    dx=wallArray[wallIdmax-3]-wallArray[j];
                    dy=wallArray[wallIdmax-2]-wallArray[j+1];
                    d2=lengarr[lengarr.size()-1]*lengarr[0];
                }else{
                    dx=wallArray[j-4]-wallArray[j];
                    dy=wallArray[j-3]-wallArray[j+1];
                    d2=lengarr[((j-wallIdmin)>>2)-1]*lengarr[(j-wallIdmin)>>2];
                }
                dx2=wallArray[j+2]-wallArray[j];
                dy2=wallArray[j+3]-wallArray[j+1];
                sinarr=fabs(-(dx*dy2-dx2*dy)/d2);
                cosarr=(dx*dx2+dy*dy2)/d2;

                //ratearr[(j-wallIdmin)>>2]=2;
                ratearr[(j-wallIdmin)>>2]=2/sinarr;
                //ratearr[(j-wallIdmin)>>2]=2/sqrt(1+cosarr);
            }

            for(j=wallIdmin;j+3<=wallIdmax;j+=4){
                //ddcyc(wallArray[j],wallArray[j+1],wallArray[j+2],wallArray[j+3],2*offset,ddArray);
                if(j==wallIdmin && l+3<leng){
                    ddcycEx(wallArray[j],wallArray[j+1],wallArray[j+2],wallArray[j+3],ratearr[(j-wallIdmin)>>2]*offset,ratearr[((j-wallIdmin)>>2)+1]*offset,ddArray);
                    m=l;
                    wayPointArrayt[l]=ddArray[0];
                    wayPointArrayt[l+1]=ddArray[1];
                    wayPointArrayt[l+2]=ddArray[2];
                    wayPointArrayt[l+3]=ddArray[3];
                    l+=4;
                }else if(j>wallIdmin && j+7<=wallIdmax && l>=2 && l+1<leng){
                    ddcycEx(wallArray[j],wallArray[j+1],wallArray[j+2],wallArray[j+3],ratearr[(j-wallIdmin)>>2]*offset,ratearr[((j-wallIdmin)>>2)+1]*offset,ddArray);
                    wayPointArrayt[l-2]=(wayPointArrayt[l-2]+ddArray[0])*0.5;
                    wayPointArrayt[l-1]=(wayPointArrayt[l-1]+ddArray[1])*0.5;
                    wayPointArrayt[l]=ddArray[2];
                    wayPointArrayt[l+1]=ddArray[3];
                    l+=2;
                }else if(j+7>wallIdmax && l>=2 && m>=0 && m+1<leng){
                    ddcycEx(wallArray[j],wallArray[j+1],wallArray[j+2],wallArray[j+3],ratearr[(j-wallIdmin)>>2]*offset,ratearr[0]*offset,ddArray);
                    wayPointArrayt[l-2]=(wayPointArrayt[l-2]+ddArray[0])*0.5;
                    wayPointArrayt[l-1]=(wayPointArrayt[l-1]+ddArray[1])*0.5;
                    wayPointArrayt[m]=(wayPointArrayt[m]+ddArray[2])*0.5;
                    wayPointArrayt[m+1]=(wayPointArrayt[m+1]+ddArray[3])*0.5;
                }
            }
        }
        if(l<(int)(wayPointArrayt.size())){
            wayPointArrayt.resize(l);
        }
    }
}

void PathFindingWidget::createWayPointColumn(bool willClearInnerWayPoints,double rmax){
    //从障碍数组之间，创建路径点数组，并在闭合三角形和四边形存在时建立多边形数组，参数是判定圆偏移量
    double roffset=0.001;
    vector <double> wayPointArrayt;
    createWayPointArray(0,&wayPointArrayt);
    if((int)(wayPointArrayt.size())<2){
        wayPointArray.clear();
        return;
    }
    wayPointArray.clear();
    columnLine.clear();
    int i=0,j=0;
    int length=(int)(wayPointArrayt.size());
    double dx,dy,cx,cy,r;
    int wl=(int)(wallArray.size());
    double dx1,dy1,dx2,dy2;
    double a1,a2,a;
    bool hasCircle=false;
    double rmin,rminj;
    double rmindx,rmindy;
    double b1,b2;
    double c1,c2;
    double r2;
    double div;
    vector<double> linemin;
    //var willCheck=atof(mainWnd->ui->thickness->text().toStdString().c_str())>0;
    vector<int> wayPointAtWall,wallAtPolygon;
    getWayPointAtWallArray(wayPointArrayt,wayPointAtWall);
    //getWallAtPolygonArray(wallAtPolygon);
    int wayPointAtWalli=0,wayPointAtWalli2=0;
    bool willRestrictR=rmax>0;

    for(i=0;i+1<length;i+=2){
        hasCircle=false;
        rmin=-1;
        for(j=0;j+1<i;j+=2){
            //if(willCheck && (i>>3)==(j>>3)){
                //continue;
            //}
            //if(!lineAllNotAcross(wayPointArrayt[j],wayPointArrayt[j+1],wayPointArrayt[i],wayPointArrayt[i+1])){
                //continue;
            //}
            dx=wayPointArrayt[i]-wayPointArrayt[j];
            dy=wayPointArrayt[i+1]-wayPointArrayt[j+1];
            cx=(wayPointArrayt[j]+wayPointArrayt[i])*0.5;
            cy=(wayPointArrayt[j+1]+wayPointArrayt[i+1])*0.5;
            r=sqrt(dx*dx+dy*dy)*0.5-roffset;
            //if(r>0 && r<rmin || rmin<=0){
                //rmin=r;
                //rmindx=-dx;
                //rmindy=-dy;
            //}

            if(r<=0 || (willRestrictR && r>rmax) || !circleAllNotAcross(cx,cy,r)){
                continue;
            }
            if(!(willClearInnerWayPoints && inClosedPolygon(cx,cy))){
                wayPointArray.push_back(cx);
                wayPointArray.push_back(cy);

                vector<double> vectort;
                vectort.resize(4);
                vectort[0]=wayPointArrayt[i];
                vectort[1]=wayPointArrayt[i+1];
                vectort[2]=wayPointArrayt[j];
                vectort[3]=wayPointArrayt[j+1];
                columnLine.push_back(vectort);
                hasCircle=true;
            }

        }
        for(j=0;j+3<wl;j+=4){
            dx=wallArray[j+2]-wallArray[j];
            dy=wallArray[j+3]-wallArray[j+1];
            dx1=wallArray[j]-wayPointArrayt[i];
            dy1=wallArray[j+1]-wayPointArrayt[i+1];
            dx2=wallArray[j+2]-wayPointArrayt[i];
            dy2=wallArray[j+3]-wayPointArrayt[i+1];
            a1=dx1*dx+dy1*dy;
            a2=dx2*dx+dy2*dy;
            if(a1*a2>=0){
                continue;
            }
            if(dx1==0 && dy1==0 || dx2==0 && dy2==0){
                continue;
            }
            a=a2/(a2-a1);
            dx=a*dx1+(1-a)*dx2;
            dy=a*dy1+(1-a)*dy2;
            cx=wayPointArrayt[i]+dx*0.5;
            cy=wayPointArrayt[i+1]+dy*0.5;
            r=sqrt(dx*dx+dy*dy)*0.5-roffset;
            if(r>0 && r<rmin || rmin<=0){
                rmin=r;
                rminj=j;
                rmindx=dx;
                rmindy=dy;
            }
            if(r<=0 || (willRestrictR && r>rmax) || !circleAllNotAcross(cx,cy,r)){
                continue;
            }
            if(!(willClearInnerWayPoints && inClosedPolygon(cx,cy))){
                wayPointArray.push_back(cx);
                wayPointArray.push_back(cy);


                vector<double> vectort;
                vectort.resize(4);
                vectort[0]=wayPointArrayt[i];
                vectort[1]=wayPointArrayt[i+1];
                vectort[2]=wayPointArrayt[i]+dx;
                vectort[3]=wayPointArrayt[i+1]+dy;
                columnLine.push_back(vectort);
                hasCircle=true;
            }

        }
        if(!hasCircle && rmin>0 && wayPointAtWall[i]>=0){
            j=rminj;
            dx2=wallArray[j+2]-wallArray[j];
            dy2=wallArray[j+3]-wallArray[j+1];
            if(wayPointAtWall[i+1]<0){
                wayPointAtWalli=wayPointAtWall[i];
                if((wayPointAtWalli>>2)==(j>>2)){
                    continue;
                }
                if((wayPointAtWalli&2)!=0){
                    dx1=wallArray[wayPointAtWalli]-wallArray[wayPointAtWalli-2];
                    dy1=wallArray[wayPointAtWalli+1]-wallArray[wayPointAtWalli-1];
                }else{
                    dx1=wallArray[wayPointAtWalli]-wallArray[wayPointAtWalli+2];
                    dy1=wallArray[wayPointAtWalli+1]-wallArray[wayPointAtWalli+3];
                }
            }else{
                //if(wallAtPolygon[wayPointAtWall[i]]==wallAtPolygon[j]){
                //    continue;
                //}
                wayPointAtWalli=wayPointAtWall[i];
                wayPointAtWalli2=wayPointAtWall[i+1];
                dx1=wallArray[wayPointAtWalli+2]-wallArray[wayPointAtWalli];
                dy1=wallArray[wayPointAtWalli+3]-wallArray[wayPointAtWalli+1];
                dx=wallArray[wayPointAtWalli2+2]-wallArray[wayPointAtWalli2];
                dy=wallArray[wayPointAtWalli2+3]-wallArray[wayPointAtWalli2+1];
                if(wayPointAtWalli2-wayPointAtWalli>4){
                    dx1=-dx1;
                    dy1=-dy1;
                }else{
                    dx=-dx;
                    dy=-dy;
                }
                a1=dx1*rmindx+dy1*rmindy;
                a2=dx*rmindx+dy*rmindy;
                if(a1*a2>=0){
                    continue;
                }
                if(a1>=0){
                    dx1=dx;
                    dy1=dy;
                }
            }

            div=0;
            a1=dy1;
            b1=-dx1;
            c1=dx1*wayPointArrayt[i+1]-dy1*wayPointArrayt[i];
            a2=dy2;
            b2=-dx2;
            c2=dx2*wallArray[j+1]-dy2*wallArray[j];
            div=a1*b2-a2*b1;

            if(div<roffset && div>-roffset){
                continue;
            }
            cx=-(c1*b2-c2*b1)/div;
            cy=-(a1*c2-a2*c1)/div;



            dx1=wayPointArrayt[i]-cx;
            dy1=wayPointArrayt[i+1]-cy;

            a=dx1*dx1+dy1*dy1;

            if(dx1*dx2+dy1*dy2>=0){
                dx=wallArray[j]-cx;
                dy=wallArray[j+1]-cy;
                a1=dx*dx2+dy*dy2>0?dx*dx+dy*dy:0;

                dx=wallArray[j+2]-cx;
                dy=wallArray[j+3]-cy;
                a2=dx*dx+dy*dy;
            }else{
                dx2=-dx2;
                dy2=-dy2;

                dx=wallArray[j]-cx;
                dy=wallArray[j+1]-cy;
                a2=dx*dx+dy*dy;

                dx=wallArray[j+2]-cx;
                dy=wallArray[j+3]-cy;
                a1=dx*dx2+dy*dy2>0?dx*dx+dy*dy:0;
            }

            if(!(a1<=a && a<=a2)){
                continue;
            }

            div=dx2*dx2+dy2*dy2;
            if(div<roffset && div>-roffset){
                continue;
            }
            r2=a;
            a=sqrt(a/div);


            dx2*=a;
            dy2*=a;


            dx=0.5*(dx1+dx2);
            dy=0.5*(dy1+dy2);
            div=dx*dx+dy*dy;
            if(div<roffset && div>-roffset){
                continue;
            }
            a=r2/div;
            dx*=a;
            dy*=a;
            r=sqrt(dx*dx+dy*dy-r2)-roffset;
            if(r>0 && (!willRestrictR || r<=rmax) && circleAllNotAcross(cx+dx,cy+dy,r) &&
               !(willClearInnerWayPoints && inClosedPolygon(cx+dx,cy+dy))){
                wayPointArray.push_back(cx+dx);
                wayPointArray.push_back(cy+dy);


                vector<double> vectort2;
                vectort2.resize(4);
                vectort2[0]=cx+dx1;
                vectort2[1]=cy+dy1;
                vectort2[2]=cx+dx;
                vectort2[3]=cy+dy;
                columnLine.push_back(vectort2);

                vector<double> vectort3;
                vectort3.resize(4);
                vectort3[0]=cx+dx;
                vectort3[1]=cy+dy;
                vectort3[2]=cx+dx2;
                vectort3[3]=cy+dy2;
                columnLine.push_back(vectort3);
            }
        }
    }
    double edgeArray2[16]={0,0,wid,0,
                    wid,0,wid,hei,
                    wid,hei,0,hei,
                    0,hei,0,0};
    for(i=0;i+1<length;i+=2){
        for(j=0;j<16;j+=4){
            dx=edgeArray2[j+2]-edgeArray2[j];
            dy=edgeArray2[j+3]-edgeArray2[j+1];
            dx1=edgeArray2[j]-wayPointArrayt[i];
            dy1=edgeArray2[j+1]-wayPointArrayt[i+1];
            dx2=edgeArray2[j+2]-wayPointArrayt[i];
            dy2=edgeArray2[j+3]-wayPointArrayt[i+1];
            a1=dx1*dx+dy1*dy;
            a2=dx2*dx+dy2*dy;
            if(a1*a2>=0){
                continue;
            }
            if(dx1==0 && dy1==0 || dx2==0 && dy2==0){
                continue;
            }
            a=a2/(a2-a1);
            dx=a*dx1+(1-a)*dx2;
            dy=a*dy1+(1-a)*dy2;
            cx=wayPointArrayt[i]+dx*0.5;
            cy=wayPointArrayt[i+1]+dy*0.5;
            r=sqrt(dx*dx+dy*dy)*0.5-roffset;
            if(r<=0 || (willRestrictR && r>rmax) || !circleAllNotAcross(cx,cy,r)){
                continue;
            }
            if(circleAcross(cx,cy,r,edgeArray2[0],edgeArray2[1],edgeArray2[2],edgeArray2[3]) ||
                circleAcross(cx,cy,r,edgeArray2[4],edgeArray2[5],edgeArray2[6],edgeArray2[7]) ||
                circleAcross(cx,cy,r,edgeArray2[8],edgeArray2[9],edgeArray2[10],edgeArray2[11]) ||
                circleAcross(cx,cy,r,edgeArray2[12],edgeArray2[13],edgeArray2[14],edgeArray2[15])){
                continue;
            }
            if(!(willClearInnerWayPoints && inClosedPolygon(cx,cy))){
                wayPointArray.push_back(cx);
                wayPointArray.push_back(cy);

                vector<double> vectort;
                vectort.resize(4);
                vectort[0]=wayPointArrayt[i];
                vectort[1]=wayPointArrayt[i+1];
                vectort[2]=wayPointArrayt[i]+dx;
                vectort[3]=wayPointArrayt[i+1]+dy;
                columnLine.push_back(vectort);}
        }
    }

    double edgeArray[8]={0,0,wid,0,wid,hei,0,hei};
    for(i=0;i<8;i+=2){
        for(j=0;j+3<wl;j+=4){
            dx=wallArray[j+2]-wallArray[j];
            dy=wallArray[j+3]-wallArray[j+1];
            dx1=wallArray[j]-edgeArray[i];
            dy1=wallArray[j+1]-edgeArray[i+1];
            dx2=wallArray[j+2]-edgeArray[i];
            dy2=wallArray[j+3]-edgeArray[i+1];
            a1=dx1*dx+dy1*dy;
            a2=dx2*dx+dy2*dy;
            if(a1*a2>=0){
                continue;
            }
            if(dx1==0 && dy1==0 || dx2==0 && dy2==0){
                continue;
            }
            a=a2/(a2-a1);
            dx=a*dx1+(1-a)*dx2;
            dy=a*dy1+(1-a)*dy2;
            cx=edgeArray[i]+dx*0.5;
            cy=edgeArray[i+1]+dy*0.5;
            r=sqrt(dx*dx+dy*dy)*0.5-roffset;
            if(r<=0 || (willRestrictR && r>rmax) || !circleAllNotAcross(cx,cy,r)){
                continue;
            }
            unddcyc(edgeArray[i],edgeArray[i+1],edgeArray[i]+dx,edgeArray[i+1]+dy,roffset,linemin);
            if(lineAllNotAcross2(linemin[0],linemin[1],linemin[2],linemin[3],columnLine)
                && !(willClearInnerWayPoints && inClosedPolygon(cx,cy))){
                wayPointArray.push_back(cx);
                wayPointArray.push_back(cy);

                vector<double> vectort;
                vectort.resize(4);
                vectort[0]=edgeArray[i];
                vectort[1]=edgeArray[i+1];
                vectort[2]=edgeArray[i]+dx;
                vectort[3]=edgeArray[i+1]+dy;
                columnLine.push_back(vectort);
            }
        }
        for(j=0;j+1<length;j+=2){
            dx=edgeArray[i]-wayPointArrayt[j];
            dy=edgeArray[i+1]-wayPointArrayt[j+1];
            cx=(wayPointArrayt[j]+edgeArray[i])*0.5;
            cy=(wayPointArrayt[j+1]+edgeArray[i+1])*0.5;
            r=sqrt(dx*dx+dy*dy)*0.5-roffset;
            if(r<=0 || (willRestrictR && r>rmax) || !circleAllNotAcross(cx,cy,r)){
                continue;
            }
            unddcyc(edgeArray[i],edgeArray[i+1],wayPointArrayt[j],wayPointArrayt[j+1],roffset,linemin);
            if(lineAllNotAcross2(linemin[0],linemin[1],linemin[2],linemin[3],columnLine)
                && !(willClearInnerWayPoints && inClosedPolygon(cx,cy))){
                wayPointArray.push_back(cx);
                wayPointArray.push_back(cy);

                vector<double> vectort;

                vectort.resize(4);
                vectort[0]=edgeArray[i];
                vectort[1]=edgeArray[i+1];
                vectort[2]=wayPointArrayt[j];
                vectort[3]=wayPointArrayt[j+1];
                columnLine.push_back(vectort);
            }
        }
    }
}

void PathFindingWidget::clearInnerWayPoints(){
    //清除在闭合的三角形或凸四边形间的路径点
    int i=0;
    int leng=(int)(wayPointArray.size());
    int j=leng;
    while(i<j){
        if(inClosedPolygon(wayPointArray[i],wayPointArray[i+1])){
            wayPointArray[i]=wayPointArray[j-2];
            wayPointArray[i+1]=wayPointArray[j-1];
            j-=2;
        }else{
            i+=2;
        }
    }
    if(j!=(int)(wayPointArray.size())){
        wayPointArray.resize(j);
    }
}

void PathFindingWidget::getWayPointAtWallArray(std::vector<double> &wayPointArrayt,std::vector<int> &wayPointAtWall){
    //获取wayPointArray到wallArray的反向对应数组（仅当用几何膨胀生成可见点时）
    int i=0,j=0,k=0;
    int length=(int)(wayPointArrayt.size());
    wayPointAtWall.resize(length);
    int polygonCount=(int)(polygonArray.size());
    int wl=(int)(wallArray.size());

    j=0;
    k=0;
    for(i=0;i+1<length;i+=2){
        if(j>=wl){
            wayPointAtWall[i]=-1;
            wayPointAtWall[i+1]=-1;
            continue;
        }
        if(k>=polygonCount){
            wayPointAtWall[i]=j;
            wayPointAtWall[i+1]=-1;
            j+=2;
            continue;
        }
        if(j<polygonArray[k]){
            wayPointAtWall[i]=j;
            wayPointAtWall[i+1]=-1;
            j+=2;
        }else if(j==polygonArray[k]){
            wayPointAtWall[i]=polygonArray[k];
            wayPointAtWall[i+1]=polygonArray[k]+4*(polygonArray[k+1]-1);
            j+=4;
        }else{
            wayPointAtWall[i]=j-4;
            wayPointAtWall[i+1]=j;
            j+=4;
            if(((j-polygonArray[k])>>2)>=polygonArray[k+1]){
                k+=2;
            }
        }
    }
}

void PathFindingWidget::getWallAtPolygonArray(std::vector<int> &wallAtPolygon){
    //获取wallArray到polygonArray的反向对应数组
    int i=0,j=0;
    int wl=(int)(wallArray.size());
    wallAtPolygon.resize(wl);
    int polygonCount=(int)(polygonArray.size());

    j=0;
    for(i=0;i+3<wl;i+=4){
        if(j>=polygonCount){
            wallAtPolygon[i]=-1;
            wallAtPolygon[i+1]=-1;
            wallAtPolygon[i+2]=-1;
            wallAtPolygon[i+3]=-1;
            continue;
        }
        if(i<polygonArray[j]){
            wallAtPolygon[i]=-1;
            wallAtPolygon[i+1]=-1;
            wallAtPolygon[i+2]=-1;
            wallAtPolygon[i+3]=-1;
        }else{
            wallAtPolygon[i]=j;
            wallAtPolygon[i+1]=j;
            wallAtPolygon[i+2]=j;
            wallAtPolygon[i+3]=j;
            if(((i-polygonArray[j])>>2)>=polygonArray[j+1]-1){
                j+=2;
            }
        }
    }
}

bool PathFindingWidget::inWalls0(double x0,double y0,int wallIdmin,int wallIdmax){
    //判断点是否在某个闭合的凸多边形中（向量外积法）
    int i=0;
    double pdx;
    double pdy;
    double ldx;
    double ldy;
    double direction=0;
    double directionPrev=direction;

    if(wallIdmax-wallIdmin<11){
        return false;
    }
    if(wallIdmin<0 || wallIdmax>=(int)(wallArray.size())){
        return false;
    }
    for(i=wallIdmin;i+3<=wallIdmax;i+=4){
        pdx=x0-wallArray[i];
        pdy=y0-wallArray[i+1];
        ldx=wallArray[i+2]-wallArray[i];
        ldy=wallArray[i+3]-wallArray[i+1];
        direction=pdx*ldy-pdy*ldx;
        if(direction>0 && directionPrev<0 || direction<0 && directionPrev>0){
            return false;
        }
        directionPrev=direction;
    }
    return true;
}

bool PathFindingWidget::inWalls(double x0,double y0,int wallIdmin,int wallIdmax){
    //判断点是否在某个闭合的凹多边形中（水平射线交点计数）
    int i=0,j=0,k=0,l=wallIdmax-wallIdmin+1;
    if(l<12){
        return false;
    }
    if(wallIdmin<0 || wallIdmax>=(int)(wallArray.size())){
        return false;
    }
    double xt=0;
    int closeCount=0;

    for(i=wallIdmin;i+3<=wallIdmax;i+=4){
        j=i+2;
        if(wallArray[i+1]>y0 && wallArray[j+1]>y0){
            continue;
        }
        if(wallArray[i+1]<y0 && wallArray[j+1]<y0){
            continue;
        }
        if(wallArray[i]>x0 && wallArray[j]>x0){
            continue;
        }
        if(wallArray[i+1]==y0){
            closeCount++;
            continue;
        }
        if(wallArray[j+1]==y0){
            k=j+5<=wallIdmax?j+4:wallIdmin;
            if((wallArray[i+1]-y0)*(wallArray[k+1]-y0)>=0){
                closeCount++;
            }
            continue;
        }
        if(wallArray[i+1]!=wallArray[j+1]){
            xt=(y0-wallArray[i+1])*(wallArray[j]-wallArray[i])/(wallArray[j+1]-wallArray[i+1])+wallArray[i];
            if(xt<=x0){
                closeCount++;
            }
        }
    }
    return (closeCount&1)!=0;
}

int PathFindingWidget::getInClosedPolygonID(double x0,double y0){
    //返回点所在的闭合的三角形或凸四边形的编号(高16位)和边数(低16位)，如果不存在则返回-1
    if(polygonArray.empty() || (int)(polygonArray.size())<2){
        return -1;
    }
    int i=0;
    int leng=(int)(polygonArray.size());
    for(i=0;i+1<leng;i+=2){
        if(inWalls(x0,y0,polygonArray[i],polygonArray[i]+4*polygonArray[i+1]-1)){
            return (polygonArray[i]<<16)|(polygonArray[i+1]&0xffff);
        }
    }
    return -1;
}

double PathFindingWidget::inClosedPolygon(double x0,double y0){
    //判断点是否在wallArray中的闭合三角形或凸四边形的内部
    return getInClosedPolygonID(x0,y0)!=-1;
}

void PathFindingWidget::ddcyc(double x0, double y0,double xt,double yt,double offset,std::vector<double> &ddcOutput){
    //端点处延长或缩短（线段坐标,线段坐标,线段坐标,线段坐标,端点处延长系数,端点处延长的输出数组）
    ddcOutput.resize(4);
    if(offset==0){
        ddcOutput[0]=x0;
        ddcOutput[1]=y0;
        ddcOutput[2]=xt;
        ddcOutput[3]=yt;
        return;
    }
    double xmid=(x0+xt)*0.5;
    double ymid=(y0+yt)*0.5;
    double dx=xt-x0;
    double dy=yt-y0;
    double d=sqrt(dx*dx+dy*dy);
    double rate=0.5*(d+2*offset)/d;
    double ratedx=rate*dx;
    double ratedy=rate*dy;
    ddcOutput[0]=xmid-ratedx;
    ddcOutput[1]=ymid-ratedy;
    ddcOutput[2]=xmid+ratedx;
    ddcOutput[3]=ymid+ratedy;
}

void PathFindingWidget::ddcycEx(double x0, double y0,double xt,double yt,double offset,double offset2,std::vector<double> &ddcOutput){
    //端点处延长或缩短（线段坐标,线段坐标,线段坐标,线段坐标,(x0,y0)端点处的延长系数,(xt,yt)端点处的延长系数,端点处延长的输出数组）
    ddcOutput.resize(4);
    if(offset==0 && offset2==0){
        ddcOutput[0]=x0;
        ddcOutput[1]=y0;
        ddcOutput[2]=xt;
        ddcOutput[3]=yt;
        return;
    }
    double xmid=(x0+xt)*0.5;
    double ymid=(y0+yt)*0.5;
    double dx=xt-x0;
    double dy=yt-y0;
    double d=sqrt(dx*dx+dy*dy);

    double rate=0.5*(d+2*offset)/d;
    ddcOutput[0]=xmid-rate*dx;
    ddcOutput[1]=ymid-rate*dy;

    rate=0.5*(d+2*offset2)/d;
    ddcOutput[2]=xmid+rate*dx;
    ddcOutput[3]=ymid+rate*dy;
}

void PathFindingWidget::unddcyc(double x0,double y0,double xt,double yt,double offset,std::vector<double> &ddcOutput){
    //端点处缩短或延长，为ddcyc方法的反向操作
    ddcyc(x0,y0,xt,yt,-offset,ddcOutput);
}

bool PathFindingWidget::lineAllNotAcross(double x0,double y0,double xt,double yt){
    //已知线段(x0,y0)到(xt,yt)与本障碍数组内的任何障碍都不相交
    int i;//循环变量
    int wl=(int)(wallArray.size());//障碍数组的长度
    for (i = 0; i+3< wl; i+=4) {
        if(lineAcross(wallArray[i], wallArray[i+1], wallArray[i+2], wallArray[i+3], x0, y0, xt, yt)){
            return false;
        }
    }
    return true;
}

bool PathFindingWidget::lineAllNotAcross2(double x0,double y0,double xt,double yt,std::vector< std::vector<double> > &arr){
    //已知线段(x0,y0)到(xt,yt)与arr数组内的任何障碍都不相交
    int i;//循环变量
    int wl=(int)(arr.size());//障碍数组的长度
    for (i = 0; i < wl; i++) {
        if(lineAcross(arr[i][0], arr[i][1], arr[i][2], arr[i][3], x0, y0, xt, yt)){
            return false;
        }
    }
    return true;
}

bool PathFindingWidget::circleAllNotAcross(double x0,double y0,double r){
    //已知以(x0,y0)为圆心，半径为r的圆与本障碍数组wallArray内的任何障碍都不相交
    int i;//循环变量
    int wl=(int)(wallArray.size());//障碍数组的长度
    for (i = 0; i < wl; i+=4) {
        if(circleAcross(x0, y0, r, wallArray[i], wallArray[i+1], wallArray[i+2], wallArray[i+3])){
            return false;
        }
    }
    return true;
}

void PathFindingWidget::createRandomWallArray(int lineCount,double xmin,double ymin,double xmax,double ymax,double width,int acronMax){
    //生成障碍包含封闭边框
    //lines——除边框外线条数
    //xmin、ymin、xmax、ymax——随机生成障碍区域


    bool retr;//线条不合，需重试。
    int acron;//相交线条数
    bool simpleTestMode=false;
    if(acronMax<=-2){
        acronMax=width<=0?1:4;
    }else if(acronMax==-1){
        acronMax=1;
        simpleTestMode=true;
    }
    int count=(width>0 && !simpleTestMode)?lineCount<<2:lineCount;
    vector< vector<double> > walls;
    walls.resize(count);

    vector< vector<double> > widen;
    //相交线条数的最大值
    int i=0,j=0;
    double a,b,c,d;


    while(i<count) {
        walls[i].resize(4);
        do {
            a = xmin+0.01+random()*(xmax-xmin);
            b = ymin+0.01+random()*(ymax-ymin);
            c = xmin+0.01+random()*(xmax-xmin);
            d = ymin+0.01+random()*(ymax-ymin);
            if(width>0 && !simpleTestMode){
                widenLine(a,b,c,d,width*0.5,widen);
                walls[i][0]=widen[0][0];
                walls[i][1]=widen[0][1];
                walls[i][2]=widen[1][0];
                walls[i][3]=widen[1][1];

                walls[i+1].resize(4);
                walls[i+1][0]=widen[1][0];
                walls[i+1][1]=widen[1][1];
                walls[i+1][2]=widen[2][0];
                walls[i+1][3]=widen[2][1];

                walls[i+2].resize(4);
                walls[i+2][0]=widen[2][0];
                walls[i+2][1]=widen[2][1];
                walls[i+2][2]=widen[3][0];
                walls[i+2][3]=widen[3][1];

                walls[i+3].resize(4);
                walls[i+3][0]=widen[3][0];
                walls[i+3][1]=widen[3][1];
                walls[i+3][2]=widen[0][0];
                walls[i+3][3]=widen[0][1];
            }else{
                walls[i][0]=a;
                walls[i][1]=b;
                walls[i][2]=c;
                walls[i][3]=d;
            }
            retr = false;
            acron = 0;
            for (j = 0; j<i; j++) {
                retr = lineAcross(walls[i][0], walls[i][1], walls[i][2], walls[i][3], walls[j][0], walls[j][1], walls[j][2], walls[j][3]);
                if (retr) {
                    acron += 1;
                }
                if (acron>acronMax) {
                    break;
                }

                if(width>0 && !simpleTestMode){
                    retr = lineAcross(walls[i+1][0], walls[i+1][1], walls[i+1][2], walls[i+1][3], walls[j][0], walls[j][1], walls[j][2], walls[j][3]);
                    if (retr) {
                        acron += 1;
                    }
                    if (acron>acronMax) {
                        break;
                    }

                    retr = lineAcross(walls[i+2][0], walls[i+2][1], walls[i+2][2], walls[i+2][3], walls[j][0], walls[j][1], walls[j][2], walls[j][3]);
                    if (retr) {
                        acron += 1;
                    }
                    if (acron>acronMax) {
                        break;
                    }

                    retr = lineAcross(walls[i+3][0], walls[i+3][1], walls[i+3][2], walls[i+3][3], walls[j][0], walls[j][1], walls[j][2], walls[j][3]);
                    if (retr) {
                        acron += 1;
                    }
                    if (acron>acronMax) {
                        break;
                    }
                }
            }
        } while (acron>acronMax);
        if(width>0 && !simpleTestMode){
            i+=4;
        }else{
            i++;
        }
    }
    //walls.push_back([xmin, ymin, xmin, ymax]);
    //walls.push_back([xmin, ymin, xmax, ymin]);
    //walls.push_back([xmax, ymin, xmax, ymax]);
    //walls.push_back([xmin, ymax, xmax, ymax]);
    int leng=0;
    if(width>0 && simpleTestMode){

        vector< vector<double> > wallsw;
        wallsw.resize(count<<2);
        for(i=0;i<count;i++){
            widenLine(walls[i][0],walls[i][1],walls[i][2],walls[i][3],width*0.5,widen);
            wallsw[i<<2].resize(4);
            wallsw[i<<2][0]=widen[0][0];
            wallsw[i<<2][1]=widen[0][1];
            wallsw[i<<2][2]=widen[1][0];
            wallsw[i<<2][3]=widen[1][1];

            wallsw[i<<2|1].resize(4);
            wallsw[i<<2|1][0]=widen[1][0];
            wallsw[i<<2|1][1]=widen[1][1];
            wallsw[i<<2|1][2]=widen[2][0];
            wallsw[i<<2|1][3]=widen[2][1];

            wallsw[i<<2|2].resize(4);
            wallsw[i<<2|2][0]=widen[2][0];
            wallsw[i<<2|2][1]=widen[2][1];
            wallsw[i<<2|2][2]=widen[3][0];
            wallsw[i<<2|2][3]=widen[3][1];

            wallsw[i<<2|3].resize(4);
            wallsw[i<<2|3][0]=widen[3][0];
            wallsw[i<<2|3][1]=widen[3][1];
            wallsw[i<<2|3][2]=widen[0][0];
            wallsw[i<<2|3][3]=widen[0][1];
        }

        leng=(int)(wallsw.size());
        if((int)(wallArray.size())!=(leng<<2)){
            wallArray.resize(leng<<2);
        }

        for(i=0;i<leng;i++){
            wallArray[i<<2]=wallsw[i][0];
            wallArray[i<<2|1]=wallsw[i][1];
            wallArray[i<<2|2]=wallsw[i][2];
            wallArray[i<<2|3]=wallsw[i][3];
        }
    }else{
        leng=(int)(walls.size());
        if((int)(wallArray.size())!=(leng<<2)){
            wallArray.resize(leng<<2);
        }

        for(i=0;i<leng;i++){
            wallArray[i<<2]=walls[i][0];
            wallArray[i<<2|1]=walls[i][1];
            wallArray[i<<2|2]=walls[i][2];
            wallArray[i<<2|3]=walls[i][3];
        }
    }
    if(width>0){
        leng=((int)(wallArray.size())>>4)<<1;
        polygonArray.resize(leng);
        for(i=0;i+1<leng;i+=2){
            polygonArray[i]=8*i;
            polygonArray[i+1]=4;
        }
        allRectObstales=true;
    }else{
        polygonArray.clear();
        allRectObstales=false;
    }
}

void PathFindingWidget::createWallArrayFromPolygon0(double d0){
    int i=0;
    while(i<(int)(polygon0.size())){
        if(polygon0[i].size()<6){
            polygon0.erase(polygon0.begin()+i);
        }else{
            i++;
        }
    }

    wallArray.clear();
    polygonArray.clear();
    allRectObstales=false;
    if(polygon0.size()==0){
        return;
    }
    int j=0,k=0,l=0,m=0,n=0,p=0;
    vector<double> lengarr;
    vector<double> sinarr;
    vector<double> cosarr;
    double anglesum=0;
    //bool oppositeDirection=false;
    double dx,dy,d2;
    double dx2,dy2;

    vector<double> arrt;
    double xt,yt;
    double range;
    double absd=fabs(d0);

    QPointF point;
    for(p=0;p<(int)(polygon0.size());p++){
        vector<double> &arr=polygon0[p];
        i=0;
        j=0;
        k=0;
        l=(int)(arr.size());
        m=0;
        n=l>>1;
        if(l<6){
            continue;
        }
        lengarr.resize(n);
        sinarr.resize(n);
        cosarr.resize(n);
        anglesum=0;
        //oppositeDirection=false;
        i=0;
        while(i+1<l){
            j=i+3<l?i+2:0;
            dx=arr[j]-arr[i];
            dy=arr[j+1]-arr[i+1];
            d2=dx*dx+dy*dy;
            if(d2==0){
                arr.erase(arr.begin()+i,arr.begin()+i+2);
                lengarr.pop_back();
                sinarr.pop_back();
                cosarr.pop_back();
                l-=2;
                n--;
                continue;
            }
            lengarr[i>>1]=sqrt(d2);
            i+=2;
        }
        i=0;
        while(i+1<l){
            j=i>=2?i-2:l-2;
            k=i+3<l?i+2:0;
            dx=arr[j]-arr[i];
            dy=arr[j+1]-arr[i+1];
            dx2=arr[k]-arr[i];
            dy2=arr[k+1]-arr[i+1];
            d2=lengarr[j>>1]*lengarr[i>>1];
            sinarr[i>>1]=-(dx*dy2-dx2*dy)/d2;//多边形沿x到y旋转方向时不用取反
            //sinarr[i>>1]=(dx*dy2-dx2*dy)/d2;//多边形沿y到x旋转方向时不用取反

            if(sinarr[i>>1]==0 && i>0){
                dx=arr[k]-arr[j];
                dy=arr[k+1]-arr[j+1];
                d2=dx*dx+dy*dy;
                lengarr[j>>1]=sqrt(d2);

                arr.erase(arr.begin()+i,arr.begin()+i+2);
                lengarr.erase(lengarr.begin()+(i>>1));
                sinarr.pop_back();
                cosarr.pop_back();
                l-=2;
                n--;
                continue;
            }else if(sinarr[i>>1]==0){
                dx=arr[k]-arr[j];
                dy=arr[k+1]-arr[j+1];
                d2=dx*dx+dy*dy;
                lengarr[0]=sqrt(d2);

                arr[0]=arr[j];
                arr[1]=arr[j+1];
                arr.pop_back();
                arr.pop_back();
                lengarr.pop_back();
                sinarr.pop_back();
                cosarr.pop_back();
                l-=2;
                n--;
                continue;
            }
            cosarr[i>>1]=(dx*dx2+dy*dy2)/d2;
            anglesum+=atan2(-sinarr[i>>1],-cosarr[i>>1]);
            i+=2;
        }
        //console.log(anglesum);
        if(anglesum>0){
            //oppositeDirection=true;
            for(m=0;m<n;m++){
                sinarr[m]=-sinarr[m];
            }
        }
        arrt.resize(l);
        m=0;
        for(i=0;i+1<l;i+=2){
            j=i>=2?i-2:l-2;
            k=i+3<l?i+2:0;
            dx=arr[j]-arr[i];
            dy=arr[j+1]-arr[i+1];
            dx2=arr[k]-arr[i];
            dy2=arr[k+1]-arr[i+1];

            xt=arr[i]-d0/sinarr[i>>1]*(dx/lengarr[j>>1]+dx2/lengarr[i>>1]);
            yt=arr[i+1]-d0/sinarr[i>>1]*(dy/lengarr[j>>1]+dy2/lengarr[i>>1]);
            if(d0*sinarr[i>>1]>=0){
                arrt[m]=xt;
                arrt[m+1]=yt;
                m+=2;
            }else{
                range=absd*(1+cosarr[i>>1])/sinarr[i>>1];
                if(range<(lengarr[j>>1]) && range<(lengarr[i>>1])){
                    arrt[m]=xt;
                    arrt[m+1]=yt;
                    m+=2;
                }
            }
        }
        if((int)(arrt.size())>m){
            arrt.resize(m);
        }
        i=4;
        while(i+1<m){
            for(j=0;j+3<i;j+=2){
                if(i+3<m && lineAcross(arrt[i],arrt[i+1],arrt[i+2],arrt[i+3],
                            arrt[j],arrt[j+1],arrt[j+2],arrt[j+3])){
                    point=crossPoint(arrt[i],arrt[i+1],arrt[i+2],arrt[i+3],
                            arrt[j],arrt[j+1],arrt[j+2],arrt[j+3]);
                    arrt[j+2]=point.x();
                    arrt[j+3]=point.y();
                    //arrt.splice(j+4,i-j-2);

                    arrt.erase(arrt.begin()+j+4,arrt.begin()+i+2);
                    m-=i-j-2;
                    i=j+2;
                    break;
                }if(i+3>=m && j>=2 && lineAcross(arrt[i],arrt[i+1],arrt[0],arrt[1],
                            arrt[j],arrt[j+1],arrt[j+2],arrt[j+3])){
                    point=crossPoint(arrt[i],arrt[i+1],arrt[0],arrt[1],
                            arrt[j],arrt[j+1],arrt[j+2],arrt[j+3]);
                    arrt[j+2]=point.x();
                    arrt[j+3]=point.y();
                    //arrt.splice(j+4,i-j-2);
                    arrt.erase(arrt.begin()+j+4,arrt.begin()+i+2);
                    m-=i-j-2;
                    i=j+2;
                    break;
                }
            }

            i+=2;
        }

        if(arrt.size()>=6){
            j=(int)(wallArray.size());
            k=(int)(arrt.size());
            polygonArray.push_back(j);
            polygonArray.push_back(k>>1);
            wallArray.resize(j+2*k);
            for(i=0;i+1<k;i+=2){
                if(i+3<(int)(arrt.size())){
                     wallArray[j+2*i]=arrt[i];
                     wallArray[j+2*i+1]=arrt[i+1];
                     wallArray[j+2*i+2]=arrt[i+2];
                     wallArray[j+2*i+3]=arrt[i+3];
                }else{
                    wallArray[j+2*i]=arrt[i];
                    wallArray[j+2*i+1]=arrt[i+1];
                    wallArray[j+2*i+2]=arrt[0];
                    wallArray[j+2*i+3]=arrt[1];
                }
            }
        }
    }
}

void PathFindingWidget::widenLine(double x0,double y0,double xt,double yt,double wid,std::vector< std::vector<double> > &wdl){
    //线段加宽，即对线段进行形态学膨胀（线段坐标，要膨胀的宽度，输出的数组）
    wdl.resize(5);
    int lineLength = sqrt((xt-x0)*(xt-x0)+(yt-y0)*(yt-y0));
    double sinA  = (yt-y0)/lineLength;
    double cosA  = (xt-x0)/lineLength;
    double sinB  = -cosA;//原线段法线的倾斜角的正弦值
    double cosB  = sinA;

    wdl[0].resize(2);
    wdl[0][0]=x0-wid*cosB-wid*cosA;
    wdl[0][1]=y0-wid*sinB-wid*sinA;

    wdl[1].resize(2);
    wdl[1][0]=x0+wid*cosB-wid*cosA;
    wdl[1][1]=y0+wid*sinB-wid*sinA;

    wdl[2].resize(2);
    wdl[2][0]=xt+wid*cosB-wid*cosA;
    wdl[2][1]=yt+wid*sinB-wid*sinA;

    wdl[3].resize(2);
    wdl[3][0]=xt-wid*cosB-wid*cosA;
    wdl[3][1]=yt-wid*sinB-wid*sinA;

    wdl[4].resize(2);
    wdl[4][0]=x0-wid*cosB-wid*cosA;
    wdl[4][1]=y0-wid*sinB-wid*sinA;
}

void PathFindingWidget::widenParallelogram(std::vector<double> &polygonArray,double wid0,std::vector< std::vector<double> > &wdl,double minWidth){
    //平行四边形加宽，即对平行四边形进行形态学膨胀（平行四边形各点坐标（长度至少为6），要膨胀的宽度，输出的数组，最小的宽度）

    if((int)(polygonArray.size())<6){
        wdl.resize(0);
        return;
    }
    wdl.resize(5);
    double dx1=(polygonArray[2]-polygonArray[0]);
    double dy1=(polygonArray[3]-polygonArray[1]);
    double dx2=(polygonArray[4]-polygonArray[2]);
    double dy2=(polygonArray[5]-polygonArray[3]);
    double aLength = sqrt(dx1*dx1+dy1*dy1);
    //平行四边形第一边的长度
    double bLength = sqrt(dx2*dx2+dy2*dy2);
    //平行四边形第二边的长度
    double abLengthMin = aLength<bLength?aLength:bLength;
    //平行四边形最短边的长度
    if(aLength==0 || bLength==0){
        wdl.resize(0);
        return;
    }
    double cosA = dx1/aLength;//平行四边形第一边的旋转角
    double sinA = dy1/aLength;
    double cosB = dx2/bLength;//平行四边形第二边的旋转角
    double sinB = dy2/bLength;
    double sinT = fabs((dx1*dy2-dx2*dy1)/(aLength*bLength));//平行四边形某个角的正弦值的绝对值
    if(sinT==0){
        wdl.resize(0);
        return;
    }
    if(0.5*abLengthMin*sinT+wid0<minWidth){
        wid0=minWidth-0.5*abLengthMin*sinT;
    }
    double wid = wid0/sinT;

    wdl[0].resize(2);
    wdl[0][0]=polygonArray[0]-wid*cosA-wid*cosB;
    wdl[0][1]=polygonArray[1]-wid*sinA-wid*sinB;

    wdl[1].resize(2);
    wdl[1][0]=polygonArray[2]+wid*cosA-wid*cosB;
    wdl[1][1]=polygonArray[3]+wid*sinA-wid*sinB;

    wdl[2].resize(2);
    wdl[2][0]=polygonArray[4]+wid*cosA+wid*cosB;
    wdl[2][1]=polygonArray[5]+wid*sinA+wid*sinB;

    if((int)(polygonArray.size())<8){
        wdl[3].resize(2);
        wdl[3][0]=polygonArray[0]+dx2-wid*cosA+wid*cosB;
        wdl[3][1]=polygonArray[1]+dy2-wid*sinA+wid*sinB;
    }else{
        wdl[3].resize(2);
        wdl[3][0]=polygonArray[6]-wid*cosA+wid*cosB;
        wdl[3][1]=polygonArray[7]-wid*sinA+wid*sinB;
    }

    wdl[4].resize(2);
    wdl[4][0]=polygonArray[0]-wid*cosA-wid*cosB;
    wdl[4][1]=polygonArray[1]-wid*sinA-wid*sinB;
}

QPointF PathFindingWidget::crossPoint(double x11,double y11,double x12,double y12,double x21,double y21,double x22,double y22){
    QPointF cp(0,0);
    double a1;
    double b1;
    double c1;
    double a2;
    double b2;
    double c2;

    a1 = y11-y12;
    b1 = x12-x11;
    c1 = x11*y12-x12*y11;
    a2 = y21-y22;
    b2 = x22-x21;
    c2 = x21*y22-x22*y21;
    double div = a1*b2-a2*b1;
    double offset=0.01;
    if(fabs(div)<offset){
        cp.setX((x12+x21)*0.5);
        cp.setY((y12+y21)*0.5);
    }else{
        cp.setX((c2*b1-c1*b2)/div);
        cp.setY((a2*c1-a1*c2)/div);
    }
    return cp;

}

bool PathFindingWidget::lineAcross(double x11,double y11,double x12,double y12,double x21,double y21,double x22,double y22){
    //向量外积法判断两线段相交。
    //排除一些特殊情形
    if(x11==x12 && y11==y12 || x21==x22 && y21==y22){
        return false;
    }

    if(x11==x21 && y11==y21 || x11==x22 && y11==y22 || x12==x21 && y12==y21 || x12==x22 && y12==y22){
        return true;
    }

    //判断第二条线是否在第一条线所在直线的同侧
    double dx1=x12-x11;
    double dy1=y12-y11;
    double dx11x21=x21-x11;
    double dy11y21=y21-y11;
    double dx11x22=x22-x11;
    double dy11y22=y22-y11;
    double d1Xd11_21=dx1*dy11y21-dy1*dx11x21;
    double d1Xd11_22=dx1*dy11y22-dy1*dx11x22;
    double offset=0.01;
    if(d1Xd11_21>offset && d1Xd11_22>offset){
        return false;
    }else if(d1Xd11_21<-offset && d1Xd11_22<-offset){
        return false;
    }
    //判断第二条线是否经过第一条线的一个端点的延长线
    double d11_21Xd11_22=dx11x21*dy11y22-dy11y21*dx11x22;
    if(d1Xd11_22<-offset && d11_21Xd11_22>offset){
        return false;
    }else if(d1Xd11_22>offset && d11_21Xd11_22<-offset){
        return false;
    }
    //判断第二条线是否经过第一条线的另一个端点的延长线
    double dx12x21=x21-x12;
    double dy12y21=y21-y12;
    double dx12x22=x22-x12;
    double dy12y22=y22-y12;
    double d12_21Xd12_22=dx12x21*dy12y22-dy12y21*dx12x22;
    if(d11_21Xd11_22<-offset && d12_21Xd12_22<-offset){
        return false;
    }else if(d11_21Xd11_22>offset && d12_21Xd12_22>offset){
        return false;
    }else{
        return true;
    }
}

bool PathFindingWidget::circleAcross(double x1,double y1,double r,double x21,double y21,double x22,double y22){
    //判断圆（闭区域）跟线段是否相交。
    //判断包围盒是否有公共部分
    double x1min=x1-r;
    double x1max=x1+r;
    double y1min=y1-r;
    double y1max=y1+r;
    double x2min;
    double x2max;
    double y2min;
    double y2max;
    if (x21<=x22) {
        x2min = x21;
        x2max = x22;
    } else {
        x2min = x22;
        x2max = x21;
    }
    if (y21<=y22) {
        y2min = y21;
        y2max = y22;
    } else {
        y2min = y22;
        y2max = y21;
    }
    if (x1max<x2min || x2max<x1min || y1max<y2min || y2max<y1min) {
        return false;
    }
    //判断线段两端点是否在圆内
    double r2=r*r;
    double dx1=x21-x1;
    double dy1=y21-y1;
    if(dx1*dx1+dy1*dy1<=r2){
        return true;
    }
    double dx2=x22-x1;
    double dy2=y22-y1;
    if(dx2*dx2+dy2*dy2<=r2){
        return true;
    }
    //判断点到线段的距离是否小于半径
    double a=y22-y21;
    double b=x21-x22;
    double c=x22*y21-x21*y22;
    double dmin2=a*x1+b*y1+c;
    dmin2=dmin2*dmin2/(a*a+b*b);
    if(dmin2>r2){
        return false;
    }
    //判断线段两端点是否在圆的两侧，即线段两端点和圆心构成的三角形中，以这个线段端点为顶点的角是否存在锐角
    double dx=x22-x21;
    double dy=y22-y21;
    if(dx*dx1+dy*dy1>0){
        return false;
    }
    if(dx*dx2+dy*dy2<0){
        return false;
    }
    return true;
}

void PathFindingWidget::updateConnection(){
    int i=0,j=0;
    vector<double> &arr0=wayPointArray;
    int count=(int)(arr0.size())>>1;
    vector< vector<int> > &arrc=connectionGraph;
    vector< vector<double> > &arrd=connectionDistance;
    arrc.resize(count);
    arrd.resize(count);
    int connectionCount=0;
    double dx,dy,d;

    for(i=0;i<count;i++){
        arrc[i].clear();
        arrd[i].clear();
    }
    for(j=1;j<count;j++){
        for(i=0;i<j;i++){
            if(!lineAllNotAcross(arr0[2*i],arr0[2*i+1],arr0[2*j],arr0[2*j+1])){
                continue;
            }
            arrc[i].push_back(j);
            arrc[j].push_back(i);

            dx=arr0[2*j]-arr0[2*i];
            dy=arr0[2*j+1]-arr0[2*i+1];
            d=sqrt(dx*dx+dy*dy);
            arrd[i].push_back(d);
            arrd[j].push_back(d);

            connectionCount+=2;
        }
    }
}

bool PathFindingWidget::findPath(double x0,double y0,double xt,double yt,vector<double> &arrpath){
    //寻路算法（起始点横坐标，起始点纵坐标，目标点横坐标，目标点纵坐标，输出的路径数组）
    arrpath.clear();
    vector< vector<int> > &arrc=connectionGraph;
    vector< vector<double> > &arrd=connectionDistance;

    if(arrc.empty() || arrd.empty()){
        updateConnection();
    }

    if(lineAllNotAcross(x0,y0,xt,yt)){
        arrpath.push_back(x0);
        arrpath.push_back(y0);
        arrpath.push_back(xt);
        arrpath.push_back(yt);
        return true;
    }

    int i=0,j=0,k=0,l=0;
    vector<double> &arrwp=wayPointArray;
    int count=(int)(arrwp.size())>>1;
    vector<double> open;
    open.clear();

    vector<int> closed,parentNode;
    vector<double> g,h;
    closed.resize(count);
    parentNode.resize(count);
    g.resize(count);
    h.resize(count);

    double dx,dy,d;
    for(i=0;i<count;i++){
        closed[i]=0;
        parentNode[i]=-2;
        dx=xt-arrwp[2*i];
        dy=yt-arrwp[2*i+1];
        d=sqrt(dx*dx+dy*dy);
        g[i]=-1;
        h[i]=d;
    }
    //var wp0=[],wpt.clear();

    double valuemin=-1;
    int valuemini=-1;
    bool willFinish=false;
    for(i=0;i<count;i++){

        if(lineAllNotAcross(arrwp[2*i],arrwp[2*i+1],xt,yt)){
            //wpt.push_back(i);
            closed[i]=-1;
        }
        if(lineAllNotAcross(x0,y0,arrwp[2*i],arrwp[2*i+1])){
            //wp0.push_back(i);
            open.push_back(i);
            parentNode[i]=-1;
            dx=arrwp[2*i]-x0;
            dy=arrwp[2*i+1]-y0;
            d=sqrt(dx*dx+dy*dy);
            g[i]=d;
        }
    }
    if((int)(open.size())<=0){
        return false;
    }
    int openCount=(int)(open.size());
    vector<double> pathNum0;
    while(openCount>0){

        valuemin=-1;
        valuemini=openCount-1;
        for(i=0;i<openCount;i++){
            j=open[i];
            if(valuemin<0 || valuemin>g[j]+h[j]){
                valuemin=g[j]+h[j];
                valuemini=i;
            }
        }

        i=open[valuemini];
        open[valuemini]=open[openCount-1];
        open.pop_back();
        willFinish=(closed[i]==-1);
        closed[i]=1;


        l=(int)(arrc[i].size());
        if(l==0){
            openCount=(int)(open.size());
            continue;
        }
        for(k=0;k<l;k++){
            j=arrc[i][k];
            if(closed[j]==1){
                continue;
            }
            //dx=arrwp[2*j]-arrwp[2*i];
            //dy=arrwp[2*j+1]-arrwp[2*i+1];
            //d=sqrt(dx*dx+dy*dy);
            d=arrd[i][k];
            if(parentNode[j]<=-2){
                parentNode[j]=i;
                g[j]=g[i]+d;
                open.push_back(j);
            }else if(parentNode[j]>=0 && g[j]>0 && g[i]+d<g[j]){
                parentNode[j]=i;
                g[j]=g[i]+d;
            }
        }
        openCount=(int)(open.size());
        if(willFinish){
            openCount++;
            pathNum0.push_back(i);
            break;
        }
    }

    if((int)(pathNum0.size())<=0){
        return false;
    }

    i=pathNum0[0];
    while(parentNode[i]>=0){
        i=parentNode[i];
        pathNum0.push_back(i);
    }

    j=(int)(pathNum0.size());
    arrpath.resize(2*j+4);
    arrpath[0]=x0;
    arrpath[1]=y0;
    for(i=1;i<=j;i++){
        k=pathNum0[j-i];
        arrpath[2*i]=arrwp[2*k];
        arrpath[2*i+1]=arrwp[2*k+1];
    }
    arrpath[2*i]=xt;
    arrpath[2*i+1]=yt;
    return true;
}

bool PathFindingWidget::rotateToAngle(double omega0,double rt,int circleNum){
    //将下标circleNum的单位在最大角速度omega下旋转到某个角度，返回应到的角度是否可达到（最大角速度，方向向量横坐标，方向向量纵坐标，原始朝向离向右顺时针旋转的角度数，单位下标）
    double r0=circleArray[circleNum][5];
    double dr=rt-r0+360;
    dr=fmod(dr,360);
    if(dr>180){
        dr-=360;
    }
    double drt=0;
    bool willArrive=false;
    if(dr>=-omega0 && dr<=omega0){
        drt=dr;
        willArrive=true;
    }else if(dr>0){
        drt=omega0;
    }else{
        drt=-omega0;
    }
    circleArray[circleNum][5]+=drt;
    return willArrive;
}

void PathFindingWidget::timerEvent(QTimerEvent *event){
    if(circleArray.empty() || !hasDest || pathArray.empty()){
        return;
    }
    int i,j;
    int count=(int)(circleArray.size());
    if(count<(int)(pathArray.size())){
        count=pathArray.size();
    }
    double dx0=0;
    double dy0=0;
    double dxj0=0;
    double dyj0=0;
    double d=0;
    double v=velocity;
    double vx0=0;
    double vy0=0;
    double vx=0;
    double vy=0;

    double r0=0;
    double interval0=0;
    double nextX;
    double nextY;


    double dxc;
    double dyc;
    double dxc0;
    double dyc0;

    double dxt=0;
    double dyt=0;
    double rc;
    double d2;

    double dxmin;
    double dymin;
    double dxmax;
    double dymax;
    int dminj;
    int dmaxj;
    double dminInterval;
    double dmaxInterval;
    double anglet;

    double anglemin;
    double anglemax;
    int stuckCount;
    int innerCount=0;
    int movedCount=0;

    double destxi,destyi;
    double destxit,destyit;
    double destXti,destYti;
    bool willFinish=false;

    bool gettingNextWayPoint=false;
    int viaCount=0;
    for(i=0;i<count;i++){
        if((int)(circleArray[i][6])==-1 || pathArray[i].empty()){
            continue;
        }

        vector <double> &pathArrayi=pathArray[i];
        if(pathArrayi.size()<2){
            continue;
        }
        destXti=pathArrayi[pathArrayi.size()-2];
        destYti=pathArrayi[pathArrayi.size()-1];
        viaCount=(int)(pathArrayi.size()>>1)-2;
        willFinish=false;

        do{
            if(lineAllNotAcross(circleArray[i][0],circleArray[i][1],destXti,destYti)){
                circleArray[i][6]=viaCount;
                willFinish=true;
                destxi=destX;
                destyi=destY;
            }else{
                willFinish=false;
                j=circleArray[i][6]+1;
                destxi=pathArrayi[2*j];
                destyi=pathArrayi[2*j+1];
            }
            dx0=destxi-circleArray[i][0];
            dy0=destyi-circleArray[i][1];
            d2=dx0*dx0+dy0*dy0;
            gettingNextWayPoint=false;
            if(!willFinish && d2<=wayPointArriveR*wayPointArriveR){
                j=circleArray[i][6]+2;
                if(j>viaCount){
                    destxit=destXti;
                    destyit=destYti;
                }else{
                    destxit=pathArrayi[2*j];
                    destyit=pathArrayi[2*j+1];
                }
                if(lineAllNotAcross(circleArray[i][0],circleArray[i][1],destxit,destyit)){
                    circleArray[i][6]+=1;
                    gettingNextWayPoint=true;
                }

            }
        }while(gettingNextWayPoint);

        if(willFinish){
            d=sqrt(d2)-dist;
        }else{
            d=sqrt(d2);
        }

        if(d<1.5*circleArray[i][2]){//(√3)r
            innerCount++;
        }
        if(d<=0){
            d=0;
            continue;
        }
        dx0=dx0*d/(d+dist);
        dy0=dy0*d/(d+dist);
        if(d>v){
            vx0=v*dx0/d;
            vy0=v*dy0/d;
        }else{
            vx0=dx0;
            vy0=dy0;
        }
        vx=vx0;
        vy=vy0;
        r0=circleArray[i][2];
        interval0=r0+interv;
        nextX=circleArray[i][0]+vx0;
        nextY=circleArray[i][1]+vy0;

        stuckCount=0;
        for(j=0;j<count;j++){
            if(j==i){
                continue;
            }
            vector<double> &arrc=circleArray[j];
            dxc=nextX-arrc[0];
            dyc=nextY-arrc[1];
            dxj0=destX-circleArray[j][0];
            dyj0=destY-circleArray[j][1];
            if(state>0 || sqrt(dxj0*dxj0+dyj0*dyj0)-dist<0.3){
                rc=arrc[2]+r0;
            }else{
                rc=arrc[2]+interval0;
            }
            d2=dxc*dxc+dyc*dyc;
            dxc0=circleArray[i][0]-arrc[0];
            dyc0=circleArray[i][1]-arrc[1];

            if(d2<=rc*rc && d2>0.3 && dxc0*vx0+dyc0*vy0<0){

                d=sqrt(dxc0*dxc0+dyc0*dyc0);
                dxt=-v*dxc0/d;
                dyt=-v*dyc0/d;
                if(stuckCount==0){
                    anglet=vx0*dyt-vy0*dxt;
                    anglemin=anglet;
                    anglemax=anglet;

                    dxmin=dxt;
                    dymin=dyt;

                    dxmax=dxt;
                    dymax=dyt;

                    dminj=j;
                    dmaxj=j;

                    dminInterval=d-arrc[2];
                    dmaxInterval=dminInterval;

                    stuckCount++;
                }else{
                    anglet=vx0*dyt-vy0*dxt;

                    if(anglet<anglemin){
                        anglemin=anglet;
                        dxmin=dxt;
                        dymin=dyt;
                        dminj=j;
                        dminInterval=d-arrc[2];
                    }
                    if(anglet>anglemax){
                        anglemax=anglet;
                        dxmax=dxt;
                        dymax=dyt;
                        dmaxj=j;
                        dmaxInterval=d-arrc[2];
                    }
                    stuckCount++;
                }
            }
        }

        if(stuckCount==0){
            circleArray[i][3]=-1;
            circleArray[i][4]=-1;
        }else if(stuckCount==1){
            //console.log(stuckCount);
            if((int)(circleArray[i][3])==dminj && multiAvoid){
                if((int)(circleArray[i][4])==0){
                    vx=-dymin;
                    vy=dxmin;
                }else if((int)(circleArray[i][4])==1){
                    vx=dymin;
                    vy=-dxmin;
                }
                if(anglemin<0 && (int)(circleArray[i][4])==0 || anglemin>=0 && (int)(circleArray[i][4])==1){
                    circleArray[i][3]=-1;
                    circleArray[i][4]=-1;
                }
            }else{
                if(anglemin>=0){
                    vx=dymin;
                    vy=-dxmin;
                }else{
                    vx=-dymin;
                    vy=dxmin;
                }
                circleArray[i][3]=-1;
                circleArray[i][4]=-1;
            }
            rc=max(r0,dminInterval);
            /*if(r0>interval0){
                //vx=vx+(rc-r0)/(interval0-r0)*(vx0-vx);
                //vy=vy+(rc-r0)/(interval0-r0)*(vy0-vy);
            }*/
            nextX=circleArray[i][0]+vx;
            nextY=circleArray[i][1]+vy;
        }else if(stuckCount>1 && multiAvoid){
            if(fabs(anglemax)>=fabs(anglemin)){
                vx=dymin;
                vy=-dxmin;
                circleArray[i][3]=dminj;
                circleArray[i][4]=1;
                rc=max(r0,dminInterval);
            }else{
                vx=-dymax;
                vy=dxmax;
                circleArray[i][3]=dmaxj;
                circleArray[i][4]=0;
                rc=max(r0,dmaxInterval);
            }
            /*if(r0>interval0){
                vx=vx+(rc-r0)/(interval0-r0)*(vx0-vx);
                vy=vy+(rc-r0)/(interval0-r0)*(vy0-vy);
            }*/
            nextX=circleArray[i][0]+vx;
            nextY=circleArray[i][1]+vy;
        }

        for(j=0;j<count;j++){
            if(j==i){
                continue;
            }
            vector<double> &arrc=circleArray[j];
            dxc=nextX-arrc[0];
            dyc=nextY-arrc[1];
            rc=arrc[2]+r0;
            d2=dxc*dxc+dyc*dyc;
            dxc0=circleArray[i][0]-arrc[0];
            dyc0=circleArray[i][1]-arrc[1];
            if(d2<=rc*rc && d2>0.3 && dxc0*vx+dyc0*vy<0){
                vx=0;
                vy=0;
                nextX=circleArray[i][0]+vx;
                nextY=circleArray[i][1]+vy;
                circleArray[i][3]=-1;
                circleArray[i][4]=-1;
                break;
            }
        }
        //circleArray[i][3]=vx;
        //circleArray[i][4]=vy;
        if(fabs(vx)>=0.001 || fabs(vy)>=0.001){
            movedCount++;
            //console.log([i,vx,vy]);
            if(rotateToAngle(omega,atan2(vy,vx)*180/PI,i)){
                if(lineAllNotAcross(circleArray[i][0],circleArray[i][1],circleArray[i][0]+vx,circleArray[i][1]+vy)){
                    circleArray[i][0]+=vx;
                    circleArray[i][1]+=vy;
                }else{// if(stuckCount==0){
                    findPath(circleArray[i][0],circleArray[i][1],destX,destY,pathArray[i]);
                    circleArray[i][6]=(!pathArray.empty())?0:-1;
                }
            }
        }
    }
    double innerCountMax=dist==0?1:(int)(PI/(asin(circleArray[0][2]/dist)));
    //console.log([innerCount,innerCountMax]);
    if(state==0 && (int)(circleArray.size())>0 && dist>0 && innerCount>=innerCountMax){
        state=1;
        //console.log(state);
    }
    update();
    if(movedCount>0){
        timeCount+=timeInterval;
    }else if(!hasTraced){
        hasTraced=true;
        ftrace();
    }
}

void PathFindingWidget::ftrace(){
    double avax=0,avay=0;
    double sx=0,sy=0;
    double dx,dy,d;
    int i=0;
    int count=(int)(circleArray.size());
    if(!hasDest || count<=0){
        return;
    }
    for(i=0;i<count;i++){
        avax+=circleArray[i][0];
        avay+=circleArray[i][1];
    }
    avax/=count;
    avay/=count;
    for(i=0;i<count;i++){
        dx=circleArray[i][0]-avax;
        dy=circleArray[i][1]-avay;
        sx+=dx*dx;
        sy+=dy*dy;
    }
    sx/=count;
    sy/=count;
    dx=destX-avax;
    dy=destY-avay;
    d=sqrt(dx*dx+dy*dy);

    double dx2=sqrt(sx);
    double dy2=sqrt(sy);
    double d2=(dx2+dy2)*0.5;
    static char str[1001];

    if(mainWnd!=NULL && traceDialog!=NULL){
        snprintf(str,1000,"average:%.3f(%.3f,%.3f)  sigma:%.3f(%.3f,%.3f)",d,dx,dy,d2,dx2,dy2);
        traceDialog->traceData(str);
    }
    //qDebug("average:%.3f(%.3f,%.3f)  sigma:%.3f(%.3f,%.3f)",d,dx,dy,d2,dx2,dy2);

}

void PathFindingWidget::ftracebtn(){
    if(mainWnd!=NULL && traceDialog==NULL){
        mainWnd->displayTraceDialog();
    }
    ftrace();
}
