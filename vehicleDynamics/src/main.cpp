#include <iostream>
#include <chrono>
#include <cstdint>
#include "collision.h"

static inline uint64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()
           ).count();
}

int main()
{
    Position center{};
    center.set(0, 0, 0, -1.5707, 0, 0);

    Cylinder cyl(1.0, 0.2, center);
    Box box(1.0, 0.9, 1.0, center);

    Position p{};
    p.set(0.0, 0.0, -0.5, 0, 0, 0);

    // 워밍업(캐시/분기예측 안정화)
    for (int i = 0; i < 10000; ++i) {
        cyl.checkcollsion(p);
        box.checkcollsion(p);
    }

    constexpr int N = 10; // 100만 (너무 작으면 1천만~1억으로 늘리기)

    // 최적화 방지용 누적(결과를 "사용"하게 만들어서 호출이 제거되지 않게)
    volatile int sink = 0;

    // Cylinder 시간 측정
    uint64_t t0 = now_ns();
    for (int i = 0; i < N; ++i) {
        cyl.checkcollsion(p);
        sink += (cyl.collision ? 1 : 0);
    }
    uint64_t t1 = now_ns();

    // Box 시간 측정
    uint64_t t2 = now_ns();
    for (int i = 0; i < N; ++i) {
        box.checkcollsion(p);
        sink += (box.collision ? 1 : 0);
    }
    uint64_t t3 = now_ns();

    double cyl_ns = double(t1 - t0) / N;
    double box_ns = double(t3 - t2) / N;

    std::cout << "Cylinder avg: " << cyl_ns << " ns/call\n";
    std::cout << "Box      avg: " << box_ns << " ns/call\n";
    std::cout << "(ignore) sink=" << sink << "\n";

    return 0;
}
