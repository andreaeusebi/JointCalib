#ifndef BASE_POINT_CLOUD_VIEWER_H
#define BASE_POINT_CLOUD_VIEWER_H

#include <QWidget>

class BasePointCloudViewer : public QWidget {
    Q_OBJECT

public:
    explicit BasePointCloudViewer(QWidget *parent = nullptr) : QWidget(parent) {}
    virtual ~BasePointCloudViewer() = default;

public slots:
    virtual void rotatePointCloud() = 0;
    virtual void translatePointCloud() = 0;
    virtual void terminateProgram() = 0;
};

#endif // BASE_POINT_CLOUD_VIEWER_H
