#ifndef IMAGEMANIPULATOR_H
#define IMAGEMANIPULATOR_H

#include <QObject>
#include <opencv2/opencv.hpp>

class ImageManipulator : public QObject
{
    Q_OBJECT


public:
    explicit ImageManipulator(QObject *parent = 0);

    void applyFilterAverage(cv::Mat &image, const size_t kernelSize);
    void applyFilterGauss(cv::Mat &image, const size_t kernelSize);
    void applyFilterLapacian(cv::Mat &image, const size_t kernelSize);
    void applyFilterSobel(cv::Mat &image, const size_t kernelSize = 3, const size_t derivative = 1);
    void applyFilterSobelManually(cv::Mat &image);
    void applyFilterCannyEdge(cv::Mat &image, const size_t threshold, const size_t ratio, const size_t kernelSize = 3);
    void applyFilterCornerHarris(cv::Mat &image, const size_t kernelSize = 3, const size_t threshold = 0);

signals:

public slots:

private:
    bool kernelSizeValid(const cv::Mat image, const size_t kernelSize);

};

#endif // IMAGEMANIPULATOR_H
