#include "camera.h"

Camera::Camera(size_t deviceID, QObject *parent) : QObject(parent), deviceID_(deviceID)
{
    // Nothing to do.
}

bool Camera::initialize()
{
    device_.open(deviceID_);
    if (!device_.isOpened()) return false;
    // get camera resolution
    imageSize_ = cv::Size(device_.get(cv::CAP_PROP_FRAME_WIDTH), device_.get(cv::CAP_PROP_FRAME_HEIGHT));
    initialized_ = true;
    return true;
}

bool Camera::initialize(size_t imageWidth, size_t imageHeight)
{
    device_.open(deviceID_);
    if (!device_.isOpened()) return false;
    // set camera resolution
    imageSize_ = cv::Size(imageWidth, imageHeight);
    device_.set(cv::CAP_PROP_FRAME_WIDTH, imageSize_.width);
    device_.set(cv::CAP_PROP_FRAME_HEIGHT, imageSize_.height);
    initialized_ = true;
    return true;
}

bool Camera::calibrate(std::vector<std::string> &filelist, cv::Size boardSize)
{
    // check for images to calibrate
    if (filelist.empty()) return false;
    // set boardSize
    boardSize_ = boardSize;
    if (!initialized_) imageSize_ = cv::imread(filelist.at(0)).size();
    // initialize object points
    initializeObjectPoints();
    // add all 2D points from images with chessboard detected
    chessboardCount_ = this->addChessboardPointsToDB(filelist);
    // check if 2D points were found
    if (!chessboardCount_) return false;
    // calculate the intrinsic parameters
    reprojectionError_ = this->calcIntrinsics();

    calibrated_ = true;
    return true;
}

void Camera::captureFrame()
{
    // check if camera is ready
    if (!initialized_) return;
    // capture frame
    this->device_ >> currentFrame_;

    if (calibrated_) {
        cv::Mat rvec, tvec;
        if (updateExtrinsics_) this->updateExtrinsics();
        if (removeDistortion_) this->removeDistortion(currentFrame_);
        if (showChessboardCorners_) this->insertChessboardPointsInImage(currentFrame_);
    }
    emit frameCaptured();
}

bool Camera::saveImage(std::string &filepath)
{
    // check if image available
    if (currentFrame_.empty()) return false;
    // save image, returns true on success
    return cv::imwrite(filepath, currentFrame_ );
}

std::vector<cv::Point2f> Camera::objectPointsToImagePoints(std::vector<cv::Point3f> &objectPoints, cv::Mat &rvec, cv::Mat &tvec)
{
    // projects 3D objectpoints to 2D imagepoints
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix_, distCoeffs_, imagePoints);
    return imagePoints;
}

cv::Vec3d Camera::eulerAnglesFromRotationVector(cv::Mat rvec)
{
    cv::Rodrigues(rvec,rvec); // converts Rotation Vector to Matrix
    cv::Mat cameraMatrix,rotMatrix,tvec,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rvec.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                             _r[3],_r[4],_r[5],0,
                             _r[6],_r[7],_r[8],0};
    cv::Vec3d eulerAngles;
    cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               tvec,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
    return eulerAngles;
}

void Camera::updateExtrinsics()
{
    std::vector<cv::Point2f> imagePoints;
    // returns true if chessboard corners were found.
    bool found =  cv::findChessboardCorners(currentFrame_, boardSize_, imagePoints,
                                            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                                            );
    if (!found) return;
    // returns true if the image corners were found.
    cv::solvePnP(objectPoints_,
                        imagePoints,
                        cameraMatrix_,
                        distCoeffs_,
                        rvec_, tvec_
                        );
    emit extrinsicsUpdated();
}

bool Camera::saveSettings(std::string &filepath)
{
    cv::FileStorage fs;
    fs.open(filepath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;

    fs << "camera_matrix" << cameraMatrix_;
    fs << "distortion_coefficients" << distCoeffs_;
    fs << "avg_reprojection_error" << reprojectionError_;

    fs.release();
    return true;
}

bool Camera::loadSettings(std::string &filepath)
{
    cv::FileStorage fs;
    fs.open(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    bool success = true;

    fs["camera_matrix"] >> cameraMatrix_;
    if (cameraMatrix_.empty()) success = false;
    fs["distortion_coefficients"] >> distCoeffs_;
    if (distCoeffs_.empty()) success = false;
    fs["avg_reprojection_error"] >> reprojectionError_;

    fs.release();
    return success;
}

void Camera::insertChessboardPointsInImage(cv::Mat &image)
{
    std::vector<cv::Point2f> imageCorners;
    // returns true if chessboard corners were found.
    bool found =  cv::findChessboardCorners(image,
                                            boardSize_,     // cols, rows
                                            imageCorners,
                                            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                                            );
    // draw the chessboard corners in the given image.
    cv::drawChessboardCorners(image, boardSize_, imageCorners, found);
}

void Camera::removeDistortion(cv::Mat &image)
{
    // initialize undistortionmaps
    if (!undistortionInitialized_) {
        cv::initUndistortRectifyMap(
            cameraMatrix_,                      // computed camera matrix
            distCoeffs_,                        // computed distortion matrix
            cv::Mat(),                          // optional rectification (none)
            cv::Mat(),                          // camera matrix to generate undistorted
            image.size(),                       // size of undistorted
            CV_16SC2,                           // type of output map
            undistortMap1_, undistortMap2_);    // x and y mapping functions
        undistortionInitialized_ = true;
    }

    // apply mapping functions
    cv::remap(image, image, undistortMap1_, undistortMap2_,
              cv::INTER_LINEAR);    // interpolation type
}

/************************** Calibration *******************************************************************/

void Camera::initializeObjectPoints()
{
    // Initialize the chessboard corners in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z) = (i,j,0)
    objectPoints_.clear();
    for (int i=0; i<boardSize_.height; ++i) {
        for (int j=0; j<boardSize_.width; ++j) {
            objectPoints_.push_back(cv::Point3f(i,j,0.0f));
        }
    }
}

size_t Camera::addChessboardPointsToDB(std::vector<std::string> &filelist)
{
    // chessboard Points
    std::vector<cv::Point2f> imageCorners;
    cv::Mat image, imageGray;
    size_t successes = 0;

    // analyze every image
    for (std::string s : filelist) {
        image = cv::imread(s);
        // get chessboard corners
        successes += cv::findChessboardCorners(image, boardSize_, imageCorners,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        // get subpixel accuracy on the corners
        cvtColor( image, imageGray, CV_BGR2GRAY );
        cv::cornerSubPix(imageGray, imageCorners, cv::Size(5,5), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
                                          40, 0.001)     // max number of iterations, minimum accuracy
                         );
        // if all corners are found on the frame, add them in the data vectors
        if (imageCorners.size() == (size_t)boardSize_.area()) {
            imagePointsDB_.push_back(imageCorners);                 // add 2D image points from one view
            objectPointsDB_.push_back(objectPoints_);   // add 3D scene points from one view
        }
    }
    return successes;
}

double Camera::calcIntrinsics()
{
    std::vector<cv::Mat> rvecs, tvecs;
    return cv::calibrateCamera(objectPointsDB_,         // 3D world points
                               imagePointsDB_,          // 2D image points
                               imageSize_,              // frame width & height
                               cameraMatrix_,           // output camera Matrix
                               distCoeffs_,             // output distortion matrix
                               rvecs, tvecs             // output rotation & translation
                               );
}

/************************** Base Functions *******************************************************************/

QImage Camera::Mat2QImage(cv::Mat &mat)
{
    cv::Mat matCopy; // make the same cv::Mat
    cv::cvtColor(mat, matCopy,CV_BGR2RGB); // deep copy
    QImage image((const uchar *) matCopy.data, matCopy.cols, matCopy.rows, matCopy.step, QImage::Format_RGB888);
    image.bits(); // enforce deep copy, see documentation
    return image;
}

cv::Mat Camera::QImage2Mat(QImage &image)
{
    cv::Mat matCopy(image.height(), image.width(), CV_8UC3, (uchar*)image.bits(), image.bytesPerLine());
    cv::Mat mat;
    cv::cvtColor(matCopy, mat,CV_BGR2RGB); // deep copy
    return mat;
}
