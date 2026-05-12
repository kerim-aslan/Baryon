#pragma once

#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include "Baryon/math/Quaternion.hpp"
#include <cstdint>

/**
 * @file CoreComponents.hpp
 * @brief Fizik motorunun temel ECS bileşenleri (Veri paketleri).
 * @details Bu dosyadaki tüm yapılar, veri odaklı tasarıma (Data-Oriented Design) uygun 
 *          olarak bellekte ardışık dizilebilen, önbellek dostu saf veri paketleri (POD) içerir.
 */
namespace Baryon::Core {

/**
 * @enum BodyType
 * @brief Fiziksel cismin temel davranış türünü belirler.
 */
enum class BodyType {
    Static,    ///< Dış güçlerden etkilenmeyen, tamamen hareketsiz cisim (örn. Zemin, Duvar).
    Kinematic, ///< Fiziksel çarpışmalara neden olan ancak sadece kod ile hareket ettirilen cisim (örn. Hareketli platform).
    Dynamic    ///< Yerçekimi, çarpışma ve dış güçlerden tamamen etkilenen fiziksel cisim.
};

/**
 * @struct Motion
 * @brief Cismin hareket hızını ve üzerindeki anlık kuvvetleri tutan bileşen.
 * @details Uygulanan dış kuvvetler ve torklar her fizik adımının (step) sonunda otomatik olarak sıfırlanır.
 */
struct Motion {
    Vector3 linearVelocity{0.0f, 0.0f, 0.0f};   ///< Cismin doğrusal hızı (m/s).
    Vector3 angularVelocity{0.0f, 0.0f, 0.0f};  ///< Cismin dönme (açısal) hızı (rad/s).
    
    Vector3 externalForce{0.0f, 0.0f, 0.0f};    ///< Bu karede cisme uygulanan toplam itme kuvveti (Newton).
    Vector3 externalTorque{0.0f, 0.0f, 0.0f};   ///< Bu karede cisme uygulanan toplam dönme kuvveti (N·m).
};

/**
 * @struct MassProps
 * @brief Cismin kütle ve dönme direncini (eylemsizlik) tanımlayan bileşen.
 * @details Statik ve Kinematik cisimler fiziksel tepki vermediği için bu cisimlerin 
 *          ters kütlesi ve ters eylemsizlik değerleri her zaman sıfır kabul edilir.
 */
struct MassProps {
    float mass{1.0f};          ///< Cismin toplam kütlesi (kg).
    float inverseMass{1.0f};   ///< Ters kütle (1 / kütle). İşlem optimizasyonu için saklanır.

    Matrix3x3 localInertiaTensor{Matrix3x3::identity()};   ///< Cismin kendi yerel eksenlerindeki dönme direnci.
    Matrix3x3 inverseInertiaTensor{Matrix3x3::identity()}; ///< Cismin dünya uzayındaki döndürülmüş ters eylemsizlik tensörü.
};

/**
 * @struct Material
 * @brief Cismin yüzey sürtünme ve esneklik özelliklerini tutan bileşen.
 */
struct Material {
    float friction{0.4f};      ///< Sürtünme katsayısı [0.0 - 1.0]. Yüksek değerler kaymayı zorlaştırır.
    float restitution{0.0000000000005f}; ///< Zıplama/Esneklik katsayısı [0.0 - 1.0]. 1.0 objenin tamamen sekmesini sağlar.
};

/**
 * @struct BodyState
 * @brief Cismin genel simülasyon durumunu ve özelliklerini tutan bileşen.
 */
struct BodyState {
    BodyType type{BodyType::Dynamic};   ///< Cismin hareket davranış tipi (Static, Kinematic, Dynamic).
    bool isSleeping{false};             ///< Cisim hareketsiz kalıp işlem tasarrufu için uyku moduna geçti mi?
    float sleepTimer{0.0f};             ///< Cismin uyku moduna geçmesi için geçen hareketsiz süre sayacı.
    bool useCCD{true};                  ///< Sürekli Çarpışma Algılama (CCD - hızlı cisimlerin tünellemesini önler) aktif mi?
    float ccdMotionThreshold{0.0f};     ///< CCD'nin devreye girmesi için cismin o karede ulaşması gereken minimum hız eşiği.
};

/**
 * @struct Linkage
 * @brief Cisimler arasındaki ebeveyn-çocuk (parent-child) hiyerarşi bağlantısı.
 * @details Bir cismin dünya pozisyonu yerine, bağlı olduğu ebeveynine göre nerede durduğunu tanımlar.
 */
struct Linkage {
    uint32_t parentId{0xFFFFFFFF};                         ///< Bağlı olduğu ebeveyn varlığın kimliği (0xFFFFFFFF = Kök/Bağımsız cisim).
    Vector3 localPosition{0.0f, 0.0f, 0.0f};               ///< Ebeveyn merkezine göre yerel konumu.
    Quaternion localOrientation{0.0f, 0.0f, 0.0f, 1.0f};   ///< Ebeveyn eksenine göre yerel dönüş (rotasyon) değeri.
};

} // namespace Baryon::Core