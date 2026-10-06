# Baryon Physics Core (Standalone C++23)

Baryon Core, herhangi bir oyun motoruna veya dış kütüphaneye bağımlılığı olmayan, **%100 saf modern C++23** ile yazılmış veri odaklı (Data-Oriented ECS) bir 3D fizik motorudur.

Bu klasörü (`baryon/`) projenize doğrudan kopyalayarak bağımsız bir fizik motoru olarak kullanabilirsiniz.

---

## 🚀 Projeye Dahil Etme (CMake)

### Yöntem 1: `add_subdirectory` ile Doğrudan Kullanım
Bu klasörü projenizin `thirdparty/` veya `libs/` dizinine kopyalayın:

```cmake
# CMakeLists.txt dosyanıza ekleyin:
add_subdirectory(baryon)

# Hedef projenize bağlayın:
target_link_libraries(MyGameEngine PRIVATE Baryon::Baryon)
```

`Baryon::Baryon` takma adı (alias) tüm include yollarını otomatik olarak projenize ekler.

---

## 📦 Klasör Yapısı
- **`include/Baryon/`**: Tüm genel API ve matematik başlık dosyaları.
  - `Simulator.hpp`: Ana simülasyon dünyası ve adım (step) yönetimi.
  - `body/Body.hpp`: Fizik nesnesi arayüzü (RigidBody, Static, Kinematic).
  - `collision/`: AABB, BVH ağacı, GJK/EPA dar faz çarpışma algoritmaları.
  - `math/`: `Vector3`, `Quaternion`, `Matrix3x3`, `Pose`.
- **`src/`**: Motorun dahili C++23 kaynak kodları.
- **`CMakeLists.txt`**: Bağımsız derleme yapılandırması.

---

## 💻 Hızlı Başlangıç Örneği

```cpp
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>

int main() {
    // 1. Simülatörü başlat (Sabit zaman adımı: 60 FPS = 0.0166s)
    Baryon::Simulator sim(1.0f / 60.0f);

    // 2. Statik zemin oluştur (100x1x100 kutu)
    Baryon::Pose groundPose(Baryon::Vector3(0, -0.5f, 0));
    Baryon::collision::CollisionShape groundShape = 
        Baryon::collision::BoxShape(Baryon::Vector3(50.0f, 0.5f, 50.0f));
    Baryon::Body ground = sim.createBody(groundPose, groundShape);
    ground.setBodyType(Baryon::Core::BodyType::Static);

    // 3. Dinamik küre oluştur (Y=10 konumunda, 1 metre yarıçapında)
    Baryon::Pose spherePose(Baryon::Vector3(0, 10.0f, 0));
    Baryon::collision::CollisionShape sphereShape = 
        Baryon::collision::SphereShape(1.0f);
    Baryon::Body sphere = sim.createBody(spherePose, sphereShape);
    sphere.setBodyType(Baryon::Core::BodyType::Dynamic);
    sphere.setMass(2.0f);

    // 4. Fizik döngüsünü çalıştır
    for (int frame = 0; frame < 60; ++frame) {
        sim.step(1.0f / 60.0f);
        auto pos = sphere.getPosition().value();
        std::cout << "Kare " << frame << " - Yükseklik: " << pos.y << std::endl;
    }

    return 0;
}
```

---

## 🛠️ Gereksinimler
- C++23 destekli modern bir derleyici:
  - GCC 13+
  - Clang 17+
  - MSVC 19.36+ (Visual Studio 2022 v17.6+)
- CMake 3.20+
