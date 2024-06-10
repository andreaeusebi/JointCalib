#ifndef BASE_POINT_CLOUD_VIEWER_H
#define BASE_POINT_CLOUD_VIEWER_H

#include <QWidget>

class BaseCloudAligner : public QWidget
{
    Q_OBJECT

    public:
        explicit BaseCloudAligner(QWidget *parent = nullptr) : QWidget(parent) {}
        virtual ~BaseCloudAligner() = default;

    public slots:
        virtual void filterMap() = 0;
        virtual void transformPointCloud() = 0;
        virtual void hideMask() = 0;
        virtual void alignMask() = 0;
        virtual void saveResults() = 0;
        virtual void terminateProgram() = 0;
};

#endif // BASE_POINT_CLOUD_VIEWER_H
