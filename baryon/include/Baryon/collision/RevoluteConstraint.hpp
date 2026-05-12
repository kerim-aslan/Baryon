#pragma once
#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"

/**
 * @file RevoluteConstraint.hpp
 * @brief Menteşe eklemi (Revolute Constraint) yapısı.
 * @details İki cismi belirli bir noktadan birbirine bağlayarak, yalnızca tek bir eksen 
 *          etrafında dönmelerine izin veren fiziksel bağlantıdır. Kapı menteşesi, 
 *          tekerlek aksı veya robot kolu gibi mekanizmalar oluşturmak için kullanılır.
 */
namespace Baryon::collision {

/**
 * @struct RevoluteConstraint
 * @brief Menteşe ekleminin fiziksel özelliklerini ve sınırlarını tutan yapı.
 */
struct RevoluteConstraint {
    ecs::Entity entityA;       ///< Birinci varlık kimliği.
    ecs::Entity entityB;       ///< İkinci varlık kimliği.

    Vector3 localAnchorA;      ///< A cismi üzerindeki yerel bağlantı (menteşe) noktası.
    Vector3 localAnchorB;      ///< B cismi üzerindeki yerel bağlantı (menteşe) noktası.

    Vector3 hingeAxisLocalA;   ///< A cisminin kendi yerel uzayındaki dönme ekseni.
    Vector3 hingeAxisLocalB;   ///< B cisminin kendi yerel uzayındaki dönme ekseni.

    bool enableLimit{false};   ///< Dönüş açısı sınırlandırmasının aktif olup olmadığı.
    float lowerAngleLimit{0.0f}; ///< İzin verilen minimum dönüş açısı (radyan cinsinden).
    float upperAngleLimit{0.0f}; ///< İzin verilen maksimum dönüş açısı (radyan cinsinden).

    /**
     * @brief Yeni bir menteşe eklemi oluşturur.
     * @param a Birinci varlık.
     * @param b İkinci varlık.
     * @param anchorA A cismi üzerindeki bağlantı noktası.
     * @param anchorB B cismi üzerindeki bağlantı noktası.
     * @param axisA A cisminin dönme ekseni (vektör otomatik olarak normalize edilir).
     * @param axisB B cisminin dönme ekseni (vektör otomatik olarak normalize edilir).
     */
    RevoluteConstraint(ecs::Entity a, ecs::Entity b, 
                  const Vector3& anchorA, const Vector3& anchorB, 
                  const Vector3& axisA, const Vector3& axisB)
        : entityA(a), entityB(b), 
          localAnchorA(anchorA), localAnchorB(anchorB), 
          hingeAxisLocalA(axisA.getNormalized()), hingeAxisLocalB(axisB.getNormalized()) {}
};

} // namespace Baryon::collision