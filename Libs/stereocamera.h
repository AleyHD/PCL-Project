#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include "camera.h"

class StereoCamera : public QObject
{
    Q_OBJECT
public:
    explicit StereoCamera(Camera *camera1, Camera *camera2, QObject *parent = 0);
    bool calibrate();
    void undistortAndRectify(cv::Mat &leftImage, cv::Mat &rightImage);
    bool saveSettings(std::string &filepath);
    bool loadSettings(std::string &filepath);
    cv::Mat createDisparityMap(cv::Mat &imgLeft, cv::Mat &imgRight);

    // inline getters
    cv::Mat fundamentalMatrix() { return fundamentalMatrix_; }
    cv::Mat essentialMatrix() { return essentialMatrix_; }
    bool calibrated() { return calibrated_; }

signals:

public slots:

private:
    std::array<Camera*, 2> cameras_;
    std::array <cv::Mat, 2> cameraMatrix_, distCoeffs_;
    double reprojectionError_ = 0.0;                              // stereo calibration error
    cv::Mat essentialMatrix_;
    cv::Mat fundamentalMatrix_;
    cv::Size imageSize_;
    std::array<cv::Mat, 2> rectifyRotation_, rectifyProjection_;
    cv::Mat rectifyReprojection_;
    bool calibrated_ = false;
    bool undistortionInitialized_ = false;
    std::array<cv::Mat, 2> undistortMapX_, undistortMapY_;

};

#endif // STEREOCAMERA_H
