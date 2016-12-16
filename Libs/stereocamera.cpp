#include "stereocamera.h"

StereoCamera::StereoCamera(Camera *camera1, Camera *camera2, QObject *parent) : QObject(parent)
{
    cameras_.at(0) = camera1;
    cameras_.at(1) = camera2;
}

bool StereoCamera::calibrate()
{
    std::array <std::vector<std::vector<cv::Point3f> >, 2> objectPoints;
    std::array <std::vector<std::vector<cv::Point2f> >, 2> imagePoints;
    std::array <cv::Size, 2> imageSize;

    for (Camera *camera : cameras_) {
        // check if cameras are calibrated
        if (!camera->calibrated()) return false;

        size_t id = camera->deviceID();
        imagePoints.at(id) = camera->imagePointsDB();
        objectPoints.at(id) = camera->objectPointsDB();
        cameraMatrix_.at(id) = camera->cameraMatrix();
        distCoeffs_.at(id) = camera->distCoeffs();
        imageSize.at(id) = camera->imageSize();
    }
    // check equal amount of images and equal resolution
    if (objectPoints.at(0).size() != objectPoints.at(1).size()) return false;
    if (imagePoints.at(0).size() != imagePoints.at(1).size()) return false;
    if (imageSize.at(0) != imageSize.at(1)) return false;

    // save resolution
    imageSize_ = imageSize.at(0);

    // preallocate
    cv::Mat rvec, tvec;
    // get stereo calibration parameters
    reprojectionError_ = cv::stereoCalibrate(
                                             objectPoints.at(0),       // 3D world points
                                             imagePoints.at(0),        // 2D image points of camera 1
                                             imagePoints.at(1),        // 2D image points of camera 2
                                             cameraMatrix_.at(0),       // in/output camera matrix of camera 1
                                             distCoeffs_.at(0),         // in/output distortion matrix of camera 1
                                             cameraMatrix_.at(1),       // in/output camera matrix of camera 2
                                             distCoeffs_.at(1),         // in/output distortion matrix of camera 2
                                             imageSize_,               // frame width & height
                                             rvec, tvec,               // output rotation & translation
                                             essentialMatrix_,         // output essential matrix
                                             fundamentalMatrix_,       // output fundamental matrix
                                             CV_CALIB_SAME_FOCAL_LENGTH +
                                             CV_CALIB_FIX_INTRINSIC +       // use input intrinsics
                                             CV_CALIB_ZERO_TANGENT_DIST,
                                             cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
                                            );
    // get stereo rectification parameters
    cv::stereoRectify(cameraMatrix_.at(0), distCoeffs_.at(0),             // input of camera 1
                      cameraMatrix_.at(1), distCoeffs_.at(1),             // input of camera 2
                      imageSize_,                                         // image resolution
                      rvec, tvec,                                         // stereoCalibrated transformation
                      rectifyRotation_.at(0), rectifyRotation_.at(1),     // output rotation matrices
                      rectifyProjection_.at(0), rectifyProjection_.at(1), // output projection matrices
                      rectifyReprojection_                                // output reprojection matrix
                     );
    calibrated_ = true;
    return true;
}

void StereoCamera::undistortAndRectify(cv::Mat &leftImage, cv::Mat &rightImage)
{
    if (!calibrated_) return;
    // initialize undistortion parameters
    if (!undistortionInitialized_) {
        for (int i=0; i<2; ++i) {
            cv::initUndistortRectifyMap(
                                        cameraMatrix_.at(i),                          // camera matrix
                                        distCoeffs_.at(i),                            // distortion matrix
                                        rectifyRotation_.at(i),                       // rotation matrix
                                        rectifyProjection_.at(i),                     // projection matrix
                                        imageSize_,                                   // size of undistorted image
                                        CV_16SC2,                                     // type of output map
                                        undistortMapX_.at(i), undistortMapY_.at(i)    // x and y mapping functions
                                       );
        }
        undistortionInitialized_ = true;
    }

    // apply mapping functions
    cv::remap(leftImage, leftImage, undistortMapX_.at(0), undistortMapY_.at(0), cv::INTER_LINEAR);
    cv::remap(rightImage, rightImage, undistortMapX_.at(1), undistortMapY_.at(1), cv::INTER_LINEAR);
}

cv::Mat StereoCamera::createDisparityMap(cv::Mat &imgLeft, cv::Mat &imgRight)
{
    if( imgLeft.empty() || imgRight.empty() ) return cv::Mat();

    // convert to gray
    cv::Mat imgGrayLeft, imgGrayRight;
    cvtColor( imgLeft, imgGrayLeft, CV_BGR2GRAY );
    cvtColor( imgRight, imgGrayRight, CV_BGR2GRAY );

    // create the image in which we will save our disparities
    cv::Mat imgDisparity16S = cv::Mat( imgLeft.rows, imgLeft.cols, CV_16S );
    cv::Mat imgDisparity8U = cv::Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );

    // call the constructor for StereoBM
    int ndisparities = 80;                // Range of disparity
    int SADWindowSize = 21;                 // Size of the block window. Must be odd
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create( ndisparities, SADWindowSize );

    // calculate the disparity image
    sbm->compute( imgGrayLeft, imgGrayRight, imgDisparity16S );

    // check its extreme values
    double minVal; double maxVal;
    minMaxLoc( imgDisparity16S, &minVal, &maxVal );

    // return it as a CV_8UC1 image
    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

    return imgDisparity8U;
}

bool StereoCamera::saveSettings(std::string &filepath)
{
    cv::FileStorage fs;
    fs.open(filepath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;

    fs << "essential_matrix" << essentialMatrix_;
    fs << "fundamental_matrix" << fundamentalMatrix_;
    fs << "avg_reprojection_error" << reprojectionError_;

    fs.release();
    return true;
}

bool StereoCamera::loadSettings(std::string &filepath)
{
    cv::FileStorage fs;
    fs.open(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    bool success = true;

    fs["essential_matrix"] >> essentialMatrix_;
    if (cameraMatrix_.empty()) success = false;
    fs["fundamental_matrix"] >> fundamentalMatrix_;
    if (distCoeffs_.empty()) success = false;
    fs["avg_reprojection_error"] >> reprojectionError_;

    fs.release();
    return success;
}
