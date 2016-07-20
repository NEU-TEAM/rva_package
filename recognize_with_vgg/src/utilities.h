#ifndef UTILITIES_H
#define UTILITIES_H

class Utilities
{
public:
    Utilities();

    static bool overlay(int minh_a, int maxh_a, int minw_a, int maxw_a, int minh_b,  int maxh_b, int minw_b, int maxw_b, int  &minh, int &maxh, int &minw, int &maxw);
};

#endif // UTILITIES_H
