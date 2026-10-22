/****************************************************************************
** Meta object code from reading C++ file 'dbcqt.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "dbcqt.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dbcqt.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QtDBCNotifier_t {
    QByteArrayData data[13];
    char stringdata0[143];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QtDBCNotifier_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QtDBCNotifier_t qt_meta_stringdata_QtDBCNotifier = {
    {
QT_MOC_LITERAL(0, 0, 13), // "QtDBCNotifier"
QT_MOC_LITERAL(1, 14, 11), // "signalAdded"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 9), // "MessageId"
QT_MOC_LITERAL(4, 37, 2), // "id"
QT_MOC_LITERAL(5, 40, 21), // "const cabana::Signal*"
QT_MOC_LITERAL(6, 62, 3), // "sig"
QT_MOC_LITERAL(7, 66, 13), // "signalRemoved"
QT_MOC_LITERAL(8, 80, 13), // "signalUpdated"
QT_MOC_LITERAL(9, 94, 10), // "msgUpdated"
QT_MOC_LITERAL(10, 105, 10), // "msgRemoved"
QT_MOC_LITERAL(11, 116, 14), // "DBCFileChanged"
QT_MOC_LITERAL(12, 131, 11) // "maskUpdated"

    },
    "QtDBCNotifier\0signalAdded\0\0MessageId\0"
    "id\0const cabana::Signal*\0sig\0signalRemoved\0"
    "signalUpdated\0msgUpdated\0msgRemoved\0"
    "DBCFileChanged\0maskUpdated"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QtDBCNotifier[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,
       7,    1,   54,    2, 0x06 /* Public */,
       8,    1,   57,    2, 0x06 /* Public */,
       9,    1,   60,    2, 0x06 /* Public */,
      10,    1,   63,    2, 0x06 /* Public */,
      11,    0,   66,    2, 0x06 /* Public */,
      12,    0,   67,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QtDBCNotifier::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QtDBCNotifier *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signalAdded((*reinterpret_cast< MessageId(*)>(_a[1])),(*reinterpret_cast< const cabana::Signal*(*)>(_a[2]))); break;
        case 1: _t->signalRemoved((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        case 2: _t->signalUpdated((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        case 3: _t->msgUpdated((*reinterpret_cast< MessageId(*)>(_a[1]))); break;
        case 4: _t->msgRemoved((*reinterpret_cast< MessageId(*)>(_a[1]))); break;
        case 5: _t->DBCFileChanged(); break;
        case 6: _t->maskUpdated(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MessageId >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MessageId >(); break;
            }
            break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MessageId >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QtDBCNotifier::*)(MessageId , const cabana::Signal * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::signalAdded)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)(const cabana::Signal * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::signalRemoved)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)(const cabana::Signal * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::signalUpdated)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)(MessageId );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::msgUpdated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)(MessageId );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::msgRemoved)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::DBCFileChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (QtDBCNotifier::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QtDBCNotifier::maskUpdated)) {
                *result = 6;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QtDBCNotifier::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_QtDBCNotifier.data,
    qt_meta_data_QtDBCNotifier,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QtDBCNotifier::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QtDBCNotifier::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QtDBCNotifier.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int QtDBCNotifier::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void QtDBCNotifier::signalAdded(MessageId _t1, const cabana::Signal * _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QtDBCNotifier::signalRemoved(const cabana::Signal * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QtDBCNotifier::signalUpdated(const cabana::Signal * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QtDBCNotifier::msgUpdated(MessageId _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void QtDBCNotifier::msgRemoved(MessageId _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void QtDBCNotifier::DBCFileChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void QtDBCNotifier::maskUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
