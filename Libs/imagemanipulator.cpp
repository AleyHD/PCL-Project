#include "imagemanipulator.h"

ImageManipulator::ImageManipulator(QObject *parent) : QObject(parent)
{

}

void ImageManipulator::applyFilterAverage(cv::Mat &image, const size_t kernelSize)
{
    // check if kernelSize und image is valid
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // initialize parameters
    cv::Point anchor = cv::Point( -1, -1 );   // anchor position relative to its kernel. Point(-1, -1) => center.
    double delta = 0;                         // value to be added to each pixel during the convolution. default 0.
    int ddepth = -1;                          // output depth. -1 => output depth = input depth.
    // Setup Kernel
    cv::Mat kernel = cv::Mat::ones(kernelSize,  kernelSize, CV_32F) / (float)(kernelSize*kernelSize);
    // Apply Filter
    cv::filter2D(image, image, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
}


void ImageManipulator::applyFilterGauss(cv::Mat &image, size_t kernelSize)
{
    // check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // setup sigma
    double sigma = (kernelSize-1)/6.0;
    // apply filter
    cv::GaussianBlur(image, image, cv::Size(kernelSize, kernelSize), sigma);
}

void ImageManipulator::applyFilterLapacian(cv::Mat &image, const size_t kernelSize)
{
    // check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // initialize parameters
    int ddepth = -1;
    // convert image to gray
    cvtColor(image, image, CV_BGR2GRAY);
    // remove noise
    cv::blur(image, image, cv::Size(kernelSize, kernelSize));
    // apply filter
    cv::Laplacian(image, image, ddepth, kernelSize);
}

void ImageManipulator::applyFilterSobel(cv::Mat &image, const size_t kernelSize, const size_t derivative)
{
    // Check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // convert image to gray
    cvtColor(image, image, CV_BGR2GRAY);
    // convert gray image to CV_32F
    image.convertTo(image, CV_32F);
    // remove noise
    cv::blur(image, image, cv::Size(kernelSize, kernelSize));
    // initialize parameters
    cv::Mat gradientX, gradientY;
    int ddepth = -1;
    // gradient X
    cv::Sobel(image, gradientX, ddepth, derivative, 0, kernelSize);
    // gradient Y
    cv::Sobel(image, gradientY, ddepth, 0, derivative, kernelSize);
    // compute gradientX^2 & gradient Y^2
    cv::Mat gradientXpow, gradientYpow;
    cv::pow(gradientX, 2, gradientXpow);
    cv::pow(gradientY, 2, gradientYpow);
    // compute gradient magnitude
    cv::Mat gradientMagnitude;
    cv::sqrt(gradientXpow+gradientYpow, gradientMagnitude);
    // convert result back in CV_8U
    cv::convertScaleAbs(gradientMagnitude, image);
}

void ImageManipulator::applyFilterSobelManually(cv::Mat &image)
{
    // Check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    // initialize parameters
    int ddepth = -1;                          // output depth. -1 => output depth = input depth.
    // convert image to gray
    cvtColor(image, image, CV_BGR2GRAY);
    // convert gray image to CV_32F
    image.convertTo(image, CV_32F);
    // remove noise
    cv::blur(image, image, cv::Size(3, 3));
    // initialize kernels
    cv::Mat kernelX = (cv::Mat_<char>(3,3) <<  1, 0, -1,
                                               2, 0, -2,
                                               1, 0, -1);
    cv::Mat kernelY = (cv::Mat_<char>(3,3) <<  1,   2,  1,
                                               0,   0,  0,
                                               -1, -2, -1);
    // initialize gradient mats
    cv::Mat gradientX, gradientY;
    // apply kernel
    cv::filter2D(image, gradientX, ddepth, kernelX);
    cv::filter2D(image, gradientY, ddepth, kernelY);
    // compute gradientX^2 & gradient Y^2
    cv::Mat gradientXpow, gradientYpow;
    cv::pow(gradientX, 2, gradientXpow);
    cv::pow(gradientY, 2, gradientYpow);
    // compute gradient magnitude
    cv::Mat gradientMagnitude;
    cv::sqrt(gradientXpow+gradientYpow, gradientMagnitude);
    // convert result back in CV_8U
    cv::convertScaleAbs(gradientMagnitude, image);
}

void ImageManipulator::applyFilterCannyEdge(cv::Mat &image, const size_t threshold, const size_t ratio, const size_t kernelSize)
{
    // Check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // convert image to gray
    cvtColor(image, image, CV_BGR2GRAY);
    // remove noise
    cv::blur(image, image, cv::Size(kernelSize, kernelSize));
    // initialize black & empty mat
    cv::Mat temp, edges;
    temp = cv::Scalar::all(0);
    // fill edges
    cv::Canny(image, edges, threshold, threshold*ratio, kernelSize);
    image.copyTo(temp, edges);
    // return edged image
    image = temp;
}

void ImageManipulator::applyFilterCornerHarris(cv::Mat &image, const size_t kernelSize, const size_t threshold)
{
    // Check if kernelSize und image is valid.
    CV_Assert(!image.empty());
    CV_Assert(kernelSizeValid(image, kernelSize));
    // convert image to gray
    cvtColor(image, image, CV_BGR2GRAY);
    // remove noise
    cv::blur(image, image, cv::Size(kernelSize, kernelSize));
    // initialize variables
    int blockSize = 2;
    double k = 0.04;
    // initialize cornerImage
    cv::Mat cornerImage ;
    // fill corners
    cv::cornerHarris(image, cornerImage, blockSize, kernelSize, k);
    // normalize between 0 and 255
    cv::normalize(cornerImage, cornerImage, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    // convert result back in CV_8U
    cv::convertScaleAbs( cornerImage, cornerImage );
    // get negative of the corner image
    cv::subtract(255, cornerImage, cornerImage);
    // show only pixel > threshold
    cv::threshold(cornerImage, cornerImage, threshold, 255, cv::THRESH_TOZERO);
    // return cornered image
    image = cornerImage;
}

bool ImageManipulator::kernelSizeValid(const cv::Mat image, const size_t kernelSize)
{
    if (kernelSize < 1) return false;
    if (!(kernelSize%2)) return false;
    if (kernelSize>(size_t)image.rows || kernelSize>(size_t)image.cols) return false;
    return true;
}
