#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include <QObject>
#include <QImage>
#include <QPainter>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>

class FeatureDetector : public QObject
{
    Q_OBJECT

public:
    enum Matcher {
        BF,
        FLANN,
    };

public:
    explicit FeatureDetector(QObject *parent = 0);

    // inline getters
    QImage feature() { return feature_; }
    qint64 detectionTime() { return detectionTime_; }

    // inline setters
    void setMatcher(Matcher matcher) { matcher_ = matcher; }


signals:
    void featureDetected();
    void error();

public slots:
    void applySIFT(QImage image1, QImage image2);
    void applySURF(QImage image1, QImage image2);
    void applySURFgpu(QImage image1, QImage image2);

private slots:
    std::vector<cv::DMatch> matchBF(cv::Mat descriptors_1, cv::Mat descriptors_2);
    std::vector<cv::DMatch> matchFLANN(cv::Mat descriptors_1, cv::Mat descriptors_2);
    std::vector<cv::DMatch> matchBFgpu(cv::cuda::GpuMat descriptors1, cv::cuda::GpuMat descriptors2);

private:
    cv::Mat QImage2Mat(QImage &image);
    QImage Mat2QImage(cv::Mat &mat);
    qint64 detectionTime_ = 0;
    QImage feature_;
    Matcher matcher_ = BF;
};

#endif // FEATUREDETECTOR_H
