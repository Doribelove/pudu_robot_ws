// Private L2 dispatch shim. The collision function itself is the unchanged
// frozen swept_rect implementation, invoked on ONE pose at a time. Adjacent
// candidates are alternatives; they are never interpreted as a trajectory.
#include <cstdint>
#include <cstddef>
using Checker = long (*)(const unsigned char*, int, int, const double*, int,
                        double, double, double, double, double);
extern "C" long first_clear_pose(std::uintptr_t address,
    const unsigned char* hard, int height, int width,
    const double* candidates, int count,
    double resolution, double ox, double oy, double length, double breadth,
    int* checked) {
    if (!address || !hard || !candidates || count < 0 || !checked) return -2;
    auto collision = reinterpret_cast<Checker>(address);
    *checked = 0;
    for (int i=0; i<count; ++i) {
        ++*checked;
        if (collision(hard,height,width,candidates+3*i,1,
                      resolution,ox,oy,length,breadth)==0) return i;
    }
    return -1;
}
