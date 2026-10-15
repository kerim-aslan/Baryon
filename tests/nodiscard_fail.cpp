#include <Baryon/math/Vector3.hpp>

int main() {
    Baryon::Vector3 a(1, 2, 3);
    Baryon::Vector3 b(4, 5, 6);
    a + b; // HATA: [[nodiscard]] donus degeri gozardi edildi
    return 0;
}
