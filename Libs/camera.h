#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <iomanip>              // std::setfill, std::setw

class Camera : public QObject
{
    Q_OBJECT

private:

public:
    explicit Camera(size_t deviceID, QObject *parent = 0);
    bool initialize();
    bool initialize(size_t imageWidth, size_t imageHeight);
    bool calibrate(std::vector<std::string> &filelist, cv::Size boardSize);
    bool loadSettings(std::string &filepath);
    bool saveSettings(std::string &filepath);
    bool saveImage(std::string &filepath);
    std::vector<cv::Point2f> objectPointsToImagePoints(std::vector<cv::Point3f> &objectPoints, cv::Mat &rvec, cv::Mat &tvec);
    cv::Vec3d eulerAnglesFromRotationVector(cv::Mat rvec);

    // inline getters
    cv::Mat cameraMatrix() { return cameraMatrix_; }
    cv::Mat distCoeffs() { return distCoeffs_; }
    double reprojectionError() { return reprojectionError_; }
    void extrinsics(cv::Mat &rvec, cv::Mat &tvec) { rvec = rvec_; tvec = tvec_; }
    size_t deviceID() { return deviceID_; }
    QImage image() { return this->Mat2QImage(currentFrame_); }
    bool initialized() { return initialized_; }
    bool calibrated() { return calibrated_; }
    std::vector<std::vector<cv::Point3f> > objectPointsDB() { return objectPointsDB_; }
    std::vector<std::vector<cv::Point2f> > imagePointsDB() {return imagePointsDB_; }
    cv::Size imageSize() { return imageSize_; }

    // inline setters
    void removeDistortion(bool decision) { removeDistortion_ = decision; }
    void showChessboardCorners(bool decision) { showChessboardCorners_ = decision; }
    void keepExtrinsicsUpdated(bool decision) { updateExtrinsics_ = decision; }

signals:
    void extrinsicsUpdated();
    void frameCaptured();

public slots:
    void captureFrame();

private:
    void updateExtrinsics();
    void removeDistortion(cv::Mat &image);
    void insertChessboardPointsInImage(cv::Mat &image);
    void initializeObjectPoints();
    size_t addChessboardPointsToDB(std::vector<std::string> &filelist);
    double calcIntrinsics();
    cv::Mat DoubleMatFromVec3b(cv::Vec3b vec);
    QImage Mat2QImage(cv::Mat &mat);
    cv::Mat QImage2Mat(QImage &image);

private:
    cv::VideoCapture device_;               // represents the physical camera
    const size_t deviceID_;                 // camera id on USB
    cv::Size imageSize_;                    // camera resolution
    cv::Mat currentFrame_;                  // current captured image
    cv::Mat cameraMatrix_;                  // intrinsic parameter
    cv::Mat distCoeffs_;                    // distortion coefficients
    cv::Mat tvec_, rvec_;                   // extrinsic parameters
    double reprojectionError_;              // intrinsic calibration error
    bool initialized_ = false;              // connection to physical device
    bool calibrated_ = false;               // camera calibrated
    // external manipulated via setters
    bool updateExtrinsics_ = false;
    bool removeDistortion_ = false;
    bool showChessboardCorners_ = false;
    // Calibration
    std::vector<cv::Point3f> objectPoints_;                    // 3D object points
    std::vector<std::vector<cv::Point3f> > objectPointsDB_;    // 3D object points DB
    std::vector<std::vector<cv::Point2f> > imagePointsDB_;     // 2D image points DB
    cv::Size boardSize_;                                       // cols, rows
    size_t imageCounter_ = 0;                                  // counts the saved images for calibration
    size_t chessboardCount_ = 0;                               // counts the images with found chessboard
    cv::Mat undistortMap1_, undistortMap2_;
    bool undistortionInitialized_ = false;
};

#endif // CAMERA_H
