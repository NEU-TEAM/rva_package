#include "request.h"

Request::Request()
{
    satisfied = false;
    requester = 0;// 0 no search, 1 no surface, 2 no object, 3 no target
}
