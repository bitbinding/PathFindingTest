/****************************************************************************
** Meta object code from reading C++ file 'pathfindingwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../PathFindingTest/pathfindingwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pathfindingwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PathFindingWidget_t {
    QByteArrayData data[12];
    char stringdata[126];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PathFindingWidget_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PathFindingWidget_t qt_meta_stringdata_PathFindingWidget = {
    {
QT_MOC_LITERAL(0, 0, 17), // "PathFindingWidget"
QT_MOC_LITERAL(1, 18, 5), // "fload"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 9), // "fgenerate"
QT_MOC_LITERAL(4, 35, 22), // "fgenerateVisiblePoints"
QT_MOC_LITERAL(5, 58, 7), // "fcustom"
QT_MOC_LITERAL(6, 66, 6), // "fspawn"
QT_MOC_LITERAL(7, 73, 7), // "fupdate"
QT_MOC_LITERAL(8, 81, 7), // "fcolumn"
QT_MOC_LITERAL(9, 89, 14), // "fnarrowDisplay"
QT_MOC_LITERAL(10, 104, 11), // "fmultiAvoid"
QT_MOC_LITERAL(11, 116, 9) // "ftracebtn"

    },
    "PathFindingWidget\0fload\0\0fgenerate\0"
    "fgenerateVisiblePoints\0fcustom\0fspawn\0"
    "fupdate\0fcolumn\0fnarrowDisplay\0"
    "fmultiAvoid\0ftracebtn"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PathFindingWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x0a /* Public */,
       3,    0,   65,    2, 0x0a /* Public */,
       4,    0,   66,    2, 0x0a /* Public */,
       5,    0,   67,    2, 0x0a /* Public */,
       6,    0,   68,    2, 0x0a /* Public */,
       7,    0,   69,    2, 0x0a /* Public */,
       8,    0,   70,    2, 0x0a /* Public */,
       9,    0,   71,    2, 0x0a /* Public */,
      10,    0,   72,    2, 0x0a /* Public */,
      11,    0,   73,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void PathFindingWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PathFindingWidget *_t = static_cast<PathFindingWidget *>(_o);
        switch (_id) {
        case 0: _t->fload(); break;
        case 1: _t->fgenerate(); break;
        case 2: _t->fgenerateVisiblePoints(); break;
        case 3: _t->fcustom(); break;
        case 4: _t->fspawn(); break;
        case 5: _t->fupdate(); break;
        case 6: _t->fcolumn(); break;
        case 7: _t->fnarrowDisplay(); break;
        case 8: _t->fmultiAvoid(); break;
        case 9: _t->ftracebtn(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject PathFindingWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PathFindingWidget.data,
      qt_meta_data_PathFindingWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PathFindingWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PathFindingWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PathFindingWidget.stringdata))
        return static_cast<void*>(const_cast< PathFindingWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PathFindingWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
