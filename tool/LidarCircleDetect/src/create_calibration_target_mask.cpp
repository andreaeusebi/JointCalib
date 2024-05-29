#include "CalibrationTargetMask.h"

int main(int argc, char **argv)
{
    CalibrationTargetMask<pcl::PointXYZRGB> calibration_mask;

    calibration_mask.visualizeAndSaveTargetMaskCloud();

    return 0;
}
