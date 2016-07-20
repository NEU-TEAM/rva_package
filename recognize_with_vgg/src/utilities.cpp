#include "utilities.h"

Utilities::Utilities()
{
}

bool Utilities::overlay(int minh_a, int maxh_a, int minw_a, int maxw_a,
                                             int minh_b, int maxh_b, int minw_b, int maxw_b,
                                             int &minh, int &maxh, int &minw, int &maxw)
{
    minh = (minh_a < minh_b) ? minh_a : minh_b;
    minw = (minw_a < minw_b) ? minw_a : minw_b;
    maxh = (maxh_a > maxh_b) ? maxh_a : maxh_b;
    maxw = (maxw_a > maxw_b) ? maxw_a : maxw_b;

    if (maxh - minh < maxh_a - minh_a + maxh_b - minh_b && maxw - minw < maxw_a - minw_a + maxw_b - minw_b)
        return true;
    else
        return false;
}
