# Baryon Physics Engine

Baryon, yüksek performanslı, veri odaklı (Data-Oriented) ve C++23 standartlarını temel alan bir 3D fizik simülasyon motorudur. Modern ECS (Entity Component System) mimarisi üzerine inşa edilmiş olup, düşük seviyeli bellek optimizasyonları ve gelişmiş çarpışma algılama algoritmaları sunar.

## Temel Özellikler

### 1. Fizik ve Simülasyon Çekirdeği

- **Sayısal Entegrasyon**: Simülasyon kararlılığını artırmak için "Semi-implicit Euler" yöntemi kullanılmıştır.
    
- **Ardışık İtme Çözücüsü (Sequential Impulses)**: Çarpışmaları ve eklem kısıtlamalarını iteratif olarak çözen, yüksek kararlılığa sahip hız tabanlı çözücü.
    
- **Ada Sistemi ve Uyku (Islands & Sleep)**: Birbiriyle etkileşim halindeki nesneleri dinamik olarak gruplandırır ve enerjisi tükenen nesneleri "uyku" moduna alarak CPU kullanımını optimize eder.
    
- **Kısıtlamalar (Constraints)**: Mesafe (Distance) ve Menteşe (Revolute) kısıtlamaları ile karmaşık mekanizmaların inşasına olanak tanır.
    

### 2. Çarpışma Algılama Sistemi

- **Geniş Faz (Broad Phase)**: Dinamik AABB Ağacı (Dynamic BVH) yapısı sayesinde binlerce nesne arasında hızlı filtreleme ve sorgulama (Raycast, AABB Query) sağlar.
    
- **Dar Faz (Narrow Phase)**: GJK (Gilbert-Johnson-Keerthi) ve EPA (Expansion Polytope Algorithm) algoritmaları ile kesin kesişim tespiti, ayırma normali ve penetrasyon derinliği hesaplaması.
    
- **Sürekli Çarpışma Algılama (CCD)**: Hızlı hareket eden mermi benzeri objelerin ince yüzeylerden "tünelleme" yaparak geçmesini engelleyen "Time of Impact" (TOI) hesaplaması.
    
- **Zengin Geometri Desteği**: Küre, Kutu, Kapsül, Dışbükey Örtü (Convex Hull) ve karmaşık Statik Üçgen Ağları (Static Mesh).
    

### 3. Modern ECS ve Bellek Mimarisi

- **SparseSet Veri Yapısı**: ECS bileşenlerini bellekte ardışık ve boşluksuz tutarak Cache-Miss oranlarını minimize eder.
    
- **PMR (Polymorphic Memory Resources)**: Uzun ömürlü veriler için Pool, geçici hesaplamalar için Monotonic bellek havuzları kullanarak çalışma zamanı tahsis maliyetlerini minimize eder.
    
- **Snapshot Sistemi**: Tüm simülasyon dünyasının durumunu kaydedip geri yükleme desteği ile Rewind ve Replay özelliklerine olanak sağlar.
    
##  Lisans

Bu projenin hakları **GPL lisansı** altındadır.