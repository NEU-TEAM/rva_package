#include "search.h"

Search::Search()
{
    minPX = 0;
    maxPX = 180;
    xStep = 30;
    minPY = 70;
    maxPY = 80;
    yStep = 10;

    cX_ = 90;
    cY_ = 70;

    count_ = 0;
    rowNum_ = (maxPY - minPY) / yStep + 1;
    colNum_ = (maxPX - minPX) / xStep + 1;
    countMax_ = rowNum_ * colNum_;
    noResult_ = false;

    cerr << "Search field: row: " << rowNum_ << " col: " << colNum_ << endl;
}

Search::Search(int minpx = 0, int maxpx = 180, int xstep = 30, int minpy = 70, int maxpy = 80, int ystep = 10, int cx = 90, int cy = 100)
{
    minPX = minpx;
    maxPX = maxpx;
    xStep = xstep;
    minPY = minpy;
    maxPY = maxpy;
    yStep = ystep;

    cX_ = cx;
    cY_ = cy;

    count_ = 0;
    rowNum_ = (maxPY - minPY) / yStep + 1;
    colNum_ = (maxPX - minPX) / xStep + 1;
    countMax_ = rowNum_ * colNum_;
    noResult_ = false;

    cerr << "Search field: row: " << rowNum_ << " col: " << colNum_ << endl;
}

bool Search::getNextPosition(int & px, int & py)
{
    int j = count_ / colNum_;
    int i = count_ - int (count_ / colNum_) * colNum_;
    if (j % 2 == 0)
        {
            px = maxPX - i * xStep;// left to right angle decrease
        }
    else
        {
            px = i * xStep;
        }

    py = maxPY - j * yStep;// up to down angle decrease

    count_++;

    if (count_ >= countMax_)
        {
            count_ = 0;
            noResult_ = true;
        }
    return noResult_;
}
