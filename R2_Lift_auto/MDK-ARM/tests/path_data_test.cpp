#include "path.h"
#include "path_follow.h"

int main(void)
{
    path_follow.loadPath(first_area, first_area_count);

    if (first_area_count < 3U)
    {
        return 1;
    }

    if (path_follow.getCurrentIndex() != 0)
    {
        return 2;
    }

    return 0;
}
