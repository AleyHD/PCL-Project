#include "featuredetector.h"

FeatureDetector::FeatureDetector(QObject *parent) : QObject(parent)
{

}

void FeatureDetector::applySIFT(QImage image1, QImage image2)
{   
    // start timer
    QElapsedTimer time;
    time.start();

    // convert QImage to Mat
    cv::Mat frame1 = QImage2Mat(image1);
    cv::Mat frame2 = QImage2Mat(image2);

    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();

    //-- Step 1: Detect the keypoints:
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    f2d->detect( frame1, keypoints1 );
    f2d->detect( frame2, keypoints2 );

    // error check
    if ( keypoints1.empty() || keypoints2.empty()) {
        emit error();
        return;
    }

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat descriptors1, descriptors2;
    f2d->compute( frame1, keypoints1, descriptors1 );
    f2d->compute( frame2, keypoints2, descriptors2 );

    // error check
    if ( descriptors1.empty() || descriptors2.empty()) {
        emit error();
        return;
    }

    //-- Step 3: Matching descriptor vectors
    std::vector<cv::DMatch> matches;
    if (matcher_ == BF) matches = this->matchBF(descriptors1, descriptors2);
    else if (matcher_ == FLANN) matches = this->matchFLANN(descriptors1, descriptors2);

    //-- Step 4: Draw matches
    cv::Mat matchedFrame;
    cv::drawMatches(frame1, keypoints1, frame2, keypoints2, matches, matchedFrame);

    // return result
    feature_ = Mat2QImage(matchedFrame);
    detectionTime_ = time.elapsed();
    emit featureDetected();
}

void FeatureDetector::applySURF(QImage image1, QImage image2)
{
    // start timer
    QElapsedTimer time;
    time.start();

    // convert QImage to Mat
    cv::Mat frame1 = QImage2Mat(image1);
    cv::Mat frame2 = QImage2Mat(image2);

    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create();

    cv::Mat gray1, gray2;
    cv::cvtColor(frame1, gray1, CV_BGR2GRAY);
    cv::cvtColor(frame2, gray2, CV_BGR2GRAY);

    //-- Step 1: Detect the keypoints:
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    f2d->detect( gray1, keypoints1 );
    f2d->detect( gray2, keypoints2 );

    // error check
    if ( keypoints1.empty() || keypoints2.empty()) {
        emit error();
        return;
    }

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat descriptors1, descriptors2;
    f2d->compute( gray1, keypoints1, descriptors1 );
    f2d->compute( gray2, keypoints2, descriptors2 );

    // error check
    if ( descriptors1.empty() || descriptors2.empty()) {
        emit error();
        return;
    }

    //-- Step 3: Matching descriptor vectors
    std::vector<cv::DMatch> matches;
    if (matcher_ == BF) matches = this->matchBF(descriptors1, descriptors2);
    else if (matcher_ == FLANN) matches = this->matchFLANN(descriptors1, descriptors2);

    //-- Step 4: Draw matches
    cv::Mat matchedFrame;
    cv::drawMatches(frame1, keypoints1, frame2, keypoints2, matches, matchedFrame);

    // return result
    feature_ = Mat2QImage(matchedFrame);
    detectionTime_ = time.elapsed();
    emit featureDetected();

}

void FeatureDetector::applySURFgpu(QImage image1, QImage image2)
{
    // start timer
    QElapsedTimer time;
    time.start();

    // convert QImage to Mat
    cv::Mat frame1 = QImage2Mat(image1);
    cv::Mat frame2 = QImage2Mat(image2);

    cv::Mat gray1, gray2;
    cv::cvtColor(frame1, gray1, CV_BGR2GRAY);
    cv::cvtColor(frame2, gray2, CV_BGR2GRAY);

    cv::cuda::SURF_CUDA surf;

    // Copy the image into GPU memory
    cv::cuda::GpuMat gpu1, gpu2;
    gpu1.upload( gray1 );
    gpu2.upload( gray2 );

    cv::cuda::GpuMat keypoints1gpu, keypoints2gpu; // keypoints
    cv::cuda::GpuMat descriptors1gpu, descriptors2gpu; // descriptors (features)

    //-- Steps 1 + 2, detect the keypoints and compute descriptors, both in one method
    surf( gpu1, cv::cuda::GpuMat(), keypoints1gpu, descriptors1gpu );
    surf( gpu2, cv::cuda::GpuMat(), keypoints2gpu, descriptors2gpu );

    //-- Step 3: Matching descriptor vectors using BruteForceMatcher
    std::vector<cv::DMatch> matches;
    matches = this->matchBFgpu(descriptors1gpu, descriptors2gpu);

    // Download Keypoints GPU -> CPU
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    surf.downloadKeypoints(keypoints1gpu, keypoints1);
    surf.downloadKeypoints(keypoints2gpu, keypoints2);

    // Download Descriptors GPU -> CPU
    std::vector<float> descriptors1, descriptors2;
    surf.downloadDescriptors(descriptors1gpu, descriptors1);
    surf.downloadDescriptors(descriptors2gpu, descriptors2);

    //-- Step 4: Draw matches
    cv::Mat matchedFrame;
    cv::drawMatches(frame1, keypoints1, frame2, keypoints2, matches, matchedFrame);

    // return result
    feature_ = Mat2QImage(matchedFrame);
    detectionTime_ = time.elapsed();
    emit featureDetected();

}

std::vector<cv::DMatch> FeatureDetector::matchBF(cv::Mat descriptors1, cv::Mat descriptors2)
{
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    return matches;
}

std::vector<cv::DMatch> FeatureDetector::matchFLANN(cv::Mat descriptors1, cv::Mat descriptors2)
{
    // Fast Library for Approximate Nearest Neighbors
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    return matches;
}

std::vector<cv::DMatch> FeatureDetector::matchBFgpu(cv::cuda::GpuMat descriptors1, cv::cuda::GpuMat descriptors2)
{
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher();
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    return matches;
}

QImage FeatureDetector::Mat2QImage(cv::Mat &mat)
{
    cv::Mat matCopy; // make the same cv::Mat
    cv::cvtColor(mat, matCopy,CV_BGR2RGB); // deep copy
    QImage image((const uchar *) matCopy.data, matCopy.cols, matCopy.rows, matCopy.step, QImage::Format_RGB888);
    image.bits(); // enforce deep copy, see documentation
    return image;
}

cv::Mat FeatureDetector::QImage2Mat(QImage &image)
{
    cv::Mat matCopy(image.height(), image.width(), CV_8UC3, (uchar*)image.bits(), image.bytesPerLine());
    cv::Mat mat;
    cv::cvtColor(matCopy, mat,CV_BGR2RGB); // deep copy
    return mat;
}
