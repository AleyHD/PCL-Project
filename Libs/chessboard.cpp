#include "chessboard.h"

Chessboard::Chessboard()
{

}

Chessboard::Chessboard(cv::Size boardSize)
{
    width_ = boardSize.width;
    height_ = boardSize.height;
    boardSize_ = boardSize;
}

Chessboard::Chessboard(cv::Size boardSize, size_t cubeSize)
{
    width_ = boardSize.width;
    height_ = boardSize.height;
    boardSize_ = boardSize;
    cubeSize_ = cubeSize;
}
