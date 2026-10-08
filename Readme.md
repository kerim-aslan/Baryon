# Baryon Physics Engine

> **Not:** Bu fizik motoru, modern C++ (C++23) ve oyun fiziği mimarilerini keşfetmek amacıyla **yapay zeka (AI) asistanları yardımıyla geliştirilmiş kişisel bir hobi projesidir.**

Baryon, herhangi bir dış kütüphaneye bağımlılığı olmayan, saf C++23 ile yazılmış, hafif ve veri odaklı (ECS) bir 3D fizik simülasyon motorudur.

---

## 📋 Gereksinimler

Projenin derlenebilmesi için sisteminizde aşağıdaki araçların bulunması gerekir:

- **Derleyici**: C++23 standardını tam destekleyen bir derleyici:
  - MSVC (Visual Studio 2022 v17.8+)
  - GCC 13+
  - Clang 16+
- **CMake**: Sürüm 3.20 veya üzeri
- **Git**

---

## 🛠️ Kurulum ve Derleme

### 1. Depoyu Klonlayın
```bash
git clone https://github.com/kerim-aslan/Baryon.git
cd Baryon
```

### 2. CMake ile Projeyi Yapılandırın
```bash
cmake -B build
```

> **İpucu:** Görsel Hata Ayıklayıcıyı (Visual Debugger) derlemek istemiyorsanız, sadece çekirdek motoru derlemek için:
> ```bash
> cmake -B build -DBARYON_BUILD_DEBUGGER=OFF
> ```

### 3. Derleyin
```bash
# Release modunda derleme (Tavsiye edilen):
cmake --build build --config Release

# Veya Debug modunda derleme:
cmake --build build --config Debug
```

---

## 🎮 Çalıştırma ve Testler

### 1. Görsel Teşhis Aracı (Visual Debugger & Testbed)
Simülasyon dünyasını, çarpışma gövdelerini, temas noktalarını ve anomali tahminlerini 3D arayüz üzerinden gerçek zamanlı test etmek için:

```bash
# Windows:
.\build\tools\debugger\Release\BaryonVisualDebugger.exe

# veya tekil yapılandırma çıktısında:
.\build\tools\debugger\BaryonVisualDebugger.exe
```

### 2. Otomatik Doğrulama Testleri
Motorun kararlılığını ve tünelleme (CCD) davranışlarını test etmek için:

```bash
# Tünelleme ve yüksek hız çarpışma (CCD) testleri:
.\build\tests\Release\TunnelingTest.exe

# Kapsamlı özellik ve entegrasyon testleri:
.\build\tests\Release\FeatureVerificationTest.exe
```

---

## 🔌 Kendi Projenize Dahil Etme (CMake)

Baryon Core sıfır dış bağımlılığa sahip olduğu için projenize eklemek oldukça basittir:

1. `baryon/` klasörünü projenizin `thirdparty/` veya kök dizinine ekleyin.
2. Kendi `CMakeLists.txt` dosyanıza şu satırları ekleyin:

```cmake
add_subdirectory(baryon)

# Kendi hedefinize bağlayın (Include yolları otomatik eklenir)
target_link_libraries(KendiOyununuz PRIVATE Baryon::Baryon)
```

### Minimal Kod Örneği

```cpp
#include <Baryon/Simulator.hpp>
#include <iostream>

int main() {
    // 1. Simülasyon dünyasını oluştur
    Baryon::Simulator sim;
    sim.setAccelerationField(Baryon::Vector3(0.0f, -9.81f, 0.0f)); // Yerçekimi

    // 2. Dinamik bir küre gövdesi ekle
    Baryon::Pose pose;
    pose.position = Baryon::Vector3(0.0f, 10.0f, 0.0f);
    Baryon::Body sphere = sim.createBody(pose, Baryon::collision::CollisionShape(Baryon::collision::SphereShape(1.0f)));
    sphere.setBodyType(Baryon::Core::BodyType::Dynamic);

    // 3. Simülasyonu adımla
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        std::cout << "Küre Yüksekliği: " << sphere.getPosition().y << " m\n";
    }

    return 0;
}
```

---

## 📁 Dizin Yapısı

- **`baryon/`**: Sıfır dış bağımlılığa sahip saf C++23 fizik motoru çekirdeği.
- **`debugger/`**: OpenGL & ImGui tabanlı 3D görsel hata ayıklama ve teşhis aracı.
- **`tests/`**: Otomatik CCD, tünelleme ve motor kabiliyet doğrulama testleri.
- **`Asset/`**: Testlerde kullanılan 3D modeller ve mesh varlıkları.

---

## 📜 Lisans

Bu proje **GNU General Public License v3.0 (GPLv3)** ile lisanslanmıştır. Detaylar için [LICENCE](LICENCE) dosyasına bakabilirsiniz.