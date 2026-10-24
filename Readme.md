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

### 2. Otomatik Doğrulama Testleri ve Regresyon Koşumu
Motorun kararlılığını, tünelleme (CCD) davranışlarını ve regresyon durumunu test etmek için:

```bash
# Tüm test paketini ctest ile çalıştırma:
ctest --test-dir build --output-on-failure

# Referans baseline regresyon testi (Faz 0):
.\build\BaselineRegressionTest.exe

# Headless Physics Harness (Metrik toplayıcı ve senaryo koşturucu):
.\build\BaryonHarness.exe --help
.\build\BaryonHarness.exe --list
.\build\BaryonHarness.exe --scenario slide_friction --json out.json --csv out.csv
.\build\BaryonHarness.exe --all
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
        std::cout << "Küre Yüksekliği: " << sphere.getPosition().value().y << " m\n";
    }

    return 0;
}
```

---

## 🔗 Eklem ve Kısıtlar (Joints & Constraints)

Baryon, veri odaklı (ECS uyumlu) ve sanal fonksiyonsuz sequential impulse mimarisiyle aşağıdaki eklem tiplerini destekler:

- **Mesafe ve Halat Eklemi (`DistanceConstraint`):** İki cisim arasındaki mesafeyi belirli bir hedef mesafede (`targetDistance`) tutar. Split-impulse tabanlı doğrudan pozisyon projeksiyonu (Faz 3.10) ile yerçekimi ve dinamik salınımlar altında oluşan sayısal sapma (drift) 13 mm'den 0-1.6 mm aralığına düşürülmüştür (hedef < 3 mm kriterini tam sağlar).
- **Sabit Kaynak Eklemi (`FixedConstraint`):** 6 serbestlik derecesini (3 ötelenme + 3 dönme) kilitler, kaynaklanmış katı cisimler oluşturur.
- **Küre / Bilyeli Mafsal (`BallSocketConstraint`):** 3 ötelenme serbestliğini kilitler, 3 açısal serbestliği serbest bırakır (küresel sarkaç, jiroskopik mafsal).
- **Menteşe / Döner Eklem (`RevoluteConstraint`):** Tek bir eksen etrafında dönmeye (1-DOF) izin verir, 3 ötelenme ve 2 eksen dışı eğilmeyi kilitler; açısal sınırlar (`setRevoluteLimits`), açısal motor ve tork tavanı (`setRevoluteMotor`) destekler.
- **Prizmatik Kayar Eklem (`PrismaticConstraint`):** 1 serbest doğrusal kayma eksenine izin verir (1-DOF), eksen dışı kaymaları ve dönmeleri kilitler; alt/üst sınırlar ve doğrusal motor destekler.
- **Sönümlü Yay / Yumuşak Mesafe (`SpringConstraint`):** Hooke Yasası ve sönümlü harmonik salınıcı modeline dayalı esnek mesafe kısıtı. Erin Catto soft-constraint (uyumluluk $\gamma$ ve bias $\beta$) formülasyonu ile yüksek sertlik ve büyük zaman adımlarında koşulsuz kararlı çalışır.
- **Koni-Burulma / Ragdoll Eklemi (`ConeTwistConstraint`):** 3 ötelenme serbestliğini noktasal ankorla kilitler; koni şeklinde açısal salınıma (swing cone, $\theta \le \theta_{max}$) ve birincil eksen etrafında burulmaya (axial twist, $[\psi_{min}, \psi_{max}]$) anatomik sınırlar dahilinde izin verir. İnsan omuzu, kalça mafsalı ve ragdoll karakter fiziği için kullanılır.
- **Eklem Kırılması ve Bildirim Sistemi (`Joint Breakage` & `onJointBroken`):** Tüm eklem tipleri üzerinde çizgisel kuvvet eşiği (`breakForce`), açısal tork eşiği (`breakTorque`) veya doğrudan itme eşiği (`breakImpulse`) tanımlanabilir. Ekleme binen reaksiyon kuvveti eşiği aştığında eklem otomatik kırılır (`isBroken = true; isEnabled = false;`), ada (island) sistemi kopan parçaları bağımsız adalara ayırır, gövdeler uykudan uyandırılır ve `JointBreakListener` ya da `std::function` geri çağırımı (`onJointBroken`) üzerinden bildirim fırlatılır.
- **Eksen Kilitleme / Serbestlik Derecesi Kilidi (DOF Lock / `LockAxis`):** Cisimlerin 6 serbestlik derecesi (3 doğrusal + 3 açısal) bitmask bayrakları (`LockAxis::LinearX`, `LinearY`, `LinearZ`, `AngularX`, `AngularY`, `AngularZ`, `Lock2D`, `All`) ile bağımsız kilitlenebilir. 2B oyunlar ve yandan görünümlü fizik modları için `LockAxis::Lock2D` (`LinearZ | AngularX | AngularY`), devrilmeyen dik karakter kontrolcüleri için `AngularX | AngularZ`, tek eksenli dikey asansör ve ray platformları için `LinearX | LinearZ | AngularAll` doğrudan desteklenir. Yönelimli ters kütle vektörü ($\vec{m}^{-1}$) ve dünya eylemsizlik tensörü sıfırlaması ile temas çözücüsünde tam kararlılık ve sıfır sayısal sapma (drift) sağlar.

---

## ⚡ Çözücü Kalitesi ve Kararlılık (Solver Quality & Substepping)

- **Adım İçi Alt Adımlama (Substepping / Soft-step):** Sabit fizik adımı (`dt`), yapılandırılabilir alt adımlara (`enableSubstepping = true`, `substepCount = 4`) bölünür. Çok katlı kuleler (20+ kat) ve uzun eklem zincirleri (30+ halka), düşük çözücü iterasyonuyla bile yüksek frekansta kararlı ve sarsıntısız çözülür.
- **Spekülatif Kontak (Speculative Contacts / TOI-Free CCD):** Çok hızlı hareket eden mermiler ve parçacıklar için pahalı TOI kök bulma gerektirmeyen, temas öncesi negatif penetrasyon ($C = -d$) ve hız önyargısı (`velocityBias = -d / dt`) temelli öngörülü temas mekanizması. Yaklaşma hızını cismin adım sonunda tam engel yüzeyinde duracağı şekilde ($v_n \ge -d/\Delta t$) sınırlar, pozisyon projeksiyonunda sıçrama yapmaz ($C \le 0$) ve 500 m/s hızındaki mermilerin bile 0.1m inceliğindeki duvarlardan tünellemesini kusursuz biçimde önler.
- **Jiroskopik Terim ve Dzhanibekov Etkisi (Gyroscopic Torque & RK4):** Serbest uzayda hızla dönen asimetrik katı cisimler için Euler dönme hareket denklemleri ($I \dot{\omega} + \omega \times (I\omega) = 0$). 4. Dereceden Runge-Kutta (RK4) ve sıkı kinetik enerji korunum denetimi ile sönümsüz serbest dönüşlerde açısal momentum büyüklüğü sapması $<\%0.03$ (hedef $<\%1.0$), kinetik enerji artışı tam $0$ J ve ara eylemsizlik ekseninde ($I_1 < I_2 < I_3$) Dzhanibekov taklası (tenis raketi kararsızlığı) analitik hassasiyetle simüle edilir.
- **Restitution Sapma Ayrışımı (Damping vs Velocity Threshold vs Split Impulse):** Çarpışma sekme apeksinin analitik ve deneysel ayrışımı ($e=0.8, H=5.0\text{ m}$). Düşüş ve çıkış süresince hava direncinin ($0.05$ doğrusal sönümleme) apeksi $3.29\text{ m}$'den $3.11\text{ m}$'ye düşürdüğü ($0.18\text{ m}$ kayıp), çarpışma hız eşiğinin ($1.0\text{ m/s}$) yüksek hızlı çarpışmalarda apeksi etkilemediği ve split-impulse pozisyon projeksiyonunun doğrudan hız bozmadan $0.032\text{ m}$ ayrışım sağladığı tam doğrulandı.
- **Bireysel Yerçekimi Ölçeği ve Enerji Korunumu (`gravityScale` & Damping):** Her cisim için bağımsız yerçekimi çarpanı (`body.setGravityScale(scale)`). `gravityScale = 0.0f` ile yerçekimi alanında serbest süzülme (free float), negatif değerlerle ters yerçekimi desteği. `damping = 0.0f` durumunda sürtünmesiz kinetik enerjinin yüzlerce adım boyunca $0$ J sapmayla kusursuz korunumu.

---

## 🚀 Performans ve Paralelleştirme (Multithreading & Performance)

- **Ada Bazlı Çok İş Parçacıklı Çözücü (`std::jthread` & ThreadPool):** Fiziksel etkileşim adaları (islands) tamamen bağımsız alt grafikler olduğundan `std::jthread` havuzu üzerinden eşzamanlı çözülür (`settings.threadCount = 8`). Deterministik blok dağıtımı ile 1 iş parçacığı ve 8 iş parçacığı arasında bit seviyesinde tam durum hash determinizmi (`hash_1T == hash_8T`) garanti edilir. 10,000 cisimli sahnede ada çözücüsü üzerinde **4.30x hızlanma** (341 ms/adım -> 79 ms/adım) sağlar. Çoklu eşzamanlı eklem kırılmaları iş parçacığı güvenli (thread-safe) mutex koruması ile fırlatılır.

---

## 📁 Dizin Yapısı

- **`baryon/`**: Sıfır dış bağımlılığa sahip saf C++23 fizik motoru çekirdeği.
- **`debugger/`**: OpenGL & ImGui tabanlı 3D görsel hata ayıklama ve teşhis aracı.
- **`tests/`**: Otomatik CCD, tünelleme ve motor kabiliyet doğrulama testleri.
- **`Asset/`**: Testlerde kullanılan 3D modeller ve mesh varlıkları.

---

## 📜 Lisans

Bu proje **GNU General Public License v3.0 (GPLv3)** ile lisanslanmıştır. Detaylar için [LICENSE](LICENSE) dosyasına bakabilirsiniz.