/****************************************************************************
** Meta object code from reading C++ file 'chartswidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "chartswidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'chartswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ChartsWidget_t {
    QByteArrayData data[14];
    char stringdata0[177];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ChartsWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ChartsWidget_t qt_meta_stringdata_ChartsWidget = {
    {
QT_MOC_LITERAL(0, 0, 12), // "ChartsWidget"
QT_MOC_LITERAL(1, 13, 19), // "toggleChartsDocking"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 13), // "seriesChanged"
QT_MOC_LITERAL(4, 48, 7), // "showTip"
QT_MOC_LITERAL(5, 56, 7), // "seconds"
QT_MOC_LITERAL(6, 64, 14), // "setColumnCount"
QT_MOC_LITERAL(7, 79, 1), // "n"
QT_MOC_LITERAL(8, 81, 9), // "removeAll"
QT_MOC_LITERAL(9, 91, 16), // "timeRangeChanged"
QT_MOC_LITERAL(10, 108, 40), // "std::optional<std::pair<doubl..."
QT_MOC_LITERAL(11, 149, 10), // "time_range"
QT_MOC_LITERAL(12, 160, 11), // "setIsDocked"
QT_MOC_LITERAL(13, 172, 4) // "dock"

    },
    "ChartsWidget\0toggleChartsDocking\0\0"
    "seriesChanged\0showTip\0seconds\0"
    "setColumnCount\0n\0removeAll\0timeRangeChanged\0"
    "std::optional<std::pair<double,double> >\0"
    "time_range\0setIsDocked\0dock"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ChartsWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    1,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   54,    2, 0x0a /* Public */,
       8,    0,   57,    2, 0x0a /* Public */,
       9,    1,   58,    2, 0x0a /* Public */,
      12,    1,   61,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    5,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::Bool,   13,

       0        // eod
};

void ChartsWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ChartsWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->toggleChartsDocking(); break;
        case 1: _t->seriesChanged(); break;
        case 2: _t->showTip((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->setColumnCount((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->removeAll(); break;
        case 5: _t->timeRangeChanged((*reinterpret_cast< const std::optional<std::pair<double,double> >(*)>(_a[1]))); break;
        case 6: _t->setIsDocked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ChartsWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ChartsWidget::toggleChartsDocking)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ChartsWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ChartsWidget::seriesChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ChartsWidget::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ChartsWidget::showTip)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ChartsWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_ChartsWidget.data,
    qt_meta_data_ChartsWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ChartsWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ChartsWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ChartsWidget.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int ChartsWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ChartsWidget::toggleChartsDocking()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void ChartsWidget::seriesChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void ChartsWidget::showTip(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
