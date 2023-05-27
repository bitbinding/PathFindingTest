#ifndef PATHFINDINGWIDGET_H
#define PATHFINDINGWIDGET_H

#include <QWidget>
#include "mainwindow.h"


class PathFindingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PathFindingWidget(QWidget *parent = 0);
    ~PathFindingWidget();
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void timerEvent(QTimerEvent *event);

    double random();
    void displayToPainter(QPainter *painter);
    void createWayPointArray(double offset=3,std::vector<double> *outWayPointArray=NULL);
    void createWayPointColumn(bool willClearInnerWayPoints=true,double rmax=-1);
    void clearInnerWayPoints();
    void getWayPointAtWallArray(std::vector<double> &wayPointArrayt,std::vector<int> &wayPointAtWall);
    void getWallAtPolygonArray(std::vector<int> &wallAtPolygon);
    bool inWalls0(double x0,double y0,int wallIdmin,int wallIdmax);
    bool inWalls(double x0,double y0,int wallIdmin,int wallIdmax);
    int getInClosedPolygonID(double x0,double y0);
    double inClosedPolygon(double x0,double y0);
    void ddcyc(double x0, double y0,double xt,double yt,double offset,std::vector<double> &ddcOutput);
    void ddcycEx(double x0, double y0,double xt,double yt,double offset,double offset2,std::vector<double> &ddcOutput);
    void unddcyc(double x0,double y0,double xt,double yt,double offset,std::vector<double> &ddcOutput);
    bool lineAllNotAcross(double x0,double y0,double xt,double yt);
    bool lineAllNotAcross2(double x0,double y0,double xt,double yt,std::vector< std::vector<double> > &arr);
    bool circleAllNotAcross(double x0,double y0,double r);
    void createRandomWallArray(int lineCount,double xmin,double ymin,double xmax,double ymax,double width=0,int acronMax=-2);
    void createWallArrayFromPolygon0(double d0);
    void widenLine(double x0,double y0,double xt,double yt,double wid,std::vector< std::vector<double> > &wdl);
    void widenParallelogram(std::vector<double> &polygonArray,double wid0,std::vector< std::vector<double> > &wdl,double minWidth=0.05);
    QPointF crossPoint(double x11,double y11,double x12,double y12,double x21,double y21,double x22,double y22);
    bool lineAcross(double x11,double y11,double x12,double y12,double x21,double y21,double x22,double y22);
    bool circleAcross(double x1,double y1,double r,double x21,double y21,double x22,double y22);
    void updateConnection();
    bool findPath(double x0,double y0,double xt,double yt,std::vector<double> &arrpath);
    bool rotateToAngle(double omega0,double rt,int circleNum);

    void ftrace();


    MainWindow *mainWnd;//主窗口

    double wid;//场景宽度
    double hei;//场景高度
    std::vector<double> wallArray;//碰撞检测所用的障碍数组
    std::vector<int> polygonArray;//闭合的多边形数组
    std::vector<double> wayPointArray;//基于几何膨胀生成的路径点数组
    std::vector< std::vector<double> > columnLine;//基于圆柱体可见点生成方法的参考线
    std::vector< std::vector<int> > wallGroup;//每格拥有的障碍数组

    bool willNarrow;//是否显示实际障碍（比碰撞检测所用的所用的障碍小）
    bool columnMode;//是否使用基于圆柱体的可见点生成方法
    //var unit;
    //var unitDest;
    bool hasDest;//是否存在目标点
    double destX,destY;//目标点坐标


    std::vector< std::vector<int> > connectionGraph;//连通性图结构

    std::vector< std::vector<double> > connectionDistance;//连通性图结构对应的每条线段的长度


    int wallGroupColumnCount;//障碍分区的列数
    double groupWidth;//每个障碍区的宽度
    double groupHeight;//每个障碍区的高度
    double groupOffsetX;//障碍区的x方向整体偏移
    double groupOffsetY;//障碍区的y方向整体偏移

    std::vector< std::vector<double> >pathArray;//找到的路径数组

    double wayPointArriveR;//到达中间可见点的判定距离
    std::vector< std::vector<double> > circleArray;//单位及其属性数组
    double dist;//距离目标点这个距离后停止
    double interv;//提前绕行距离
    double velocity;//单位行进速度
    double omega;//单位转动速度
    //var destX;
    int timeInterval;//时间间隔
    bool multiAvoid;//绕行多个单位

    int state;//行进状态
    int timeCount;//行进时间
    bool hasTraced;//已经做出了数据跟踪
    bool drawingObstales;//是否正在绘制障碍
    std::vector< std::vector<double> > polygon0;//原始的多边形数组
    bool allRectObstales;//是否全部为矩形障碍
signals:

public slots:
    void fload();
    void fgenerate();
    void fgenerateVisiblePoints();
    void fcustom();
    void fspawn();
    void fupdate();
    void fcolumn();
    void fnarrowDisplay();
    void fmultiAvoid();
    void ftracebtn();
};

#endif // PATHFINDINGWIDGET_H
