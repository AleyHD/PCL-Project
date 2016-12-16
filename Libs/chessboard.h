#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include <opencv2/opencv.hpp>

class Chessboard
{
public:
    Chessboard();
    Chessboard(cv::Size boardSize);
    Chessboard(cv::Size boardSize, size_t cubeSize);

    // inline setters
    void setWidth(size_t width) { width_ = width; }
    void setHeight(size_t height) { height_ = height; }
    void setCubeSize(size_t cubeSize) {cubeSize_ = cubeSize; }

    // inline getters
    cv::Size boardSize() { return boardSize_; }
    size_t width() { return width_; }
    size_t height() { return height_; }
    size_t cubeSize() { return cubeSize_; }

private:
    cv::Size boardSize_;
    size_t width_;
    size_t height_;
    size_t cubeSize_;
};

#endif // CHESSBOARD_H
