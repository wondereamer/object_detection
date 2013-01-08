/****************************************************************************
** Meta object code from reading C++ file 'efpisoft.h'
**
** Created: Fri Jan 4 15:38:26 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "efpisoft.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'efpisoft.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainCanvas[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      25,   11,   11,   11, 0x05,
      38,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      62,   11,   11,   11, 0x0a,
      69,   11,   11,   11, 0x0a,
      82,   11,   11,   11, 0x0a,
      95,   11,   11,   11, 0x0a,
     107,   11,   11,   11, 0x0a,
     120,   11,   11,   11, 0x0a,
     129,   11,   11,   11, 0x0a,
     145,   11,   11,   11, 0x0a,
     167,   11,   11,   11, 0x0a,
     189,   11,   11,   11, 0x0a,
     207,   11,   11,   11, 0x0a,
     224,   11,   11,   11, 0x0a,
     232,   11,   11,   11, 0x0a,
     248,   11,   11,   11, 0x0a,
     261,   11,   11,   11, 0x0a,
     278,   11,   11,   11, 0x0a,
     297,   11,   11,   11, 0x0a,
     317,   11,   11,   11, 0x2a,

       0        // eod
};

static const char qt_meta_stringdata_MainCanvas[] = {
    "MainCanvas\0\0tinChanged()\0hfpChanged()\0"
    "numclustersChanged(int)\0open()\0"
    "save_model()\0save_scene()\0closeMesh()\0"
    "properties()\0runhfp()\0shuffleColors()\0"
    "increaseNumClusters()\0decreaseNumClusters()\0"
    "showAllClusters()\0showOneCluster()\0"
    "about()\0setSceneGraph()\0checkNoTIN()\0"
    "checkHFPStatus()\0updateMatIndexes()\0"
    "setNumClusters(int)\0setNumClusters()\0"
};

const QMetaObject MainCanvas::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainCanvas,
      qt_meta_data_MainCanvas, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainCanvas::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainCanvas::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainCanvas::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainCanvas))
        return static_cast<void*>(const_cast< MainCanvas*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainCanvas::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: tinChanged(); break;
        case 1: hfpChanged(); break;
        case 2: numclustersChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: open(); break;
        case 4: save_model(); break;
        case 5: save_scene(); break;
        case 6: closeMesh(); break;
        case 7: properties(); break;
        case 8: runhfp(); break;
        case 9: shuffleColors(); break;
        case 10: increaseNumClusters(); break;
        case 11: decreaseNumClusters(); break;
        case 12: showAllClusters(); break;
        case 13: showOneCluster(); break;
        case 14: about(); break;
        case 15: setSceneGraph(); break;
        case 16: checkNoTIN(); break;
        case 17: checkHFPStatus(); break;
        case 18: updateMatIndexes(); break;
        case 19: setNumClusters((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 20: setNumClusters(); break;
        default: ;
        }
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void MainCanvas::tinChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void MainCanvas::hfpChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void MainCanvas::numclustersChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
