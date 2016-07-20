#ifndef SEARCH_H
#define SEARCH_H

#include <iostream>

using namespace std;

class Search
{
public:
    Search();
    Search(int minpx, int maxpx, int xstep, int minpy, int maxpy, int ystep, int cx, int cy);

    bool getNextPosition(int & px, int & py);

    inline void getCenter(int & cx, int & cy)
    {
        cx = cX_;
        cy = cY_;
    }

    inline void setCurrentPosition(int px, int py)
    {
        pos_x = px;
        pos_y = py;
    }

    inline void resetSearch()
    {
        count_ = 0;
        noResult_ = false;
    }

private:
    int pos_x;
    int pos_y;

    int minPX;
    int maxPX;
    int xStep;
    int minPY;
    int maxPY;
    int yStep;

    int cX_;
    int cY_;

    int count_;
    int rowNum_;
    int colNum_;
    int countMax_;

    bool noResult_;
};

#endif // SEARCH_H
