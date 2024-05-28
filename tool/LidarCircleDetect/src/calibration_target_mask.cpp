#include "CalibrationTargetMask.h"

int main(int argc, char **argv)
{
    CalibrationTargetMask<pcl::PointXYZ> calibration_mask;

    calibration_mask.visualizeTargetMaskCloud();

    return 0;
}