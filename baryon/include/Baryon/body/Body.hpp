#pragma once

#include "Baryon/Core/Registry.hpp"
#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Quaternion.hpp"
#include <expected>

/**
 * @file Body.hpp
 * @brief Fiziksel cisim (Body) kullanıcı API'si.
 * ECS varlıklarını ve bileşenlerini güvenli bir şekilde yönetmek için kullanılır.
 */
namespace Baryon {

/**
 * @brief Fizik işlemlerinde oluşabilecek hata kodları.
 */
enum class PhysicsError {
    EntityDead,       ///< Varlık silinmiş veya geçersiz.
    ComponentMissing  ///< Varlıkta istenilen bileşen bulunamadı.
};

/**
 * @class Body
 * @brief Fiziksel cisimlere güvenli erişim sağlayan sarmalayıcı sınıf.
 * ECS kayıt defteri üzerinden veri erişimi ve std::expected ile hata yönetimi sağlar.
 */
class Body {
private:
    ecs::Entity mEntity;       ///< Sarmalanan varlığın kimliği.
    Core::Registry* mRegistry; ///< ECS kayıt defterine işaretçi.

public:
    /**
     * @brief Body sarmalayıcısı oluşturur.
     * @param entity Sarmalanacak varlık.
     * @param registry ECS kayıt defteri.
     */
    Body(ecs::Entity entity, Core::Registry& registry)
        : mEntity(entity), mRegistry(&registry) {}

    /** @brief Sarmalanan varlığın kimliğini (Entity ID) döndürür. */
    [[nodiscard]] ecs::Entity getEntity() const { return mEntity; }

    /** @brief Varlığın sistemde hala aktif olup olmadığını kontrol eder. */
    [[nodiscard]] bool isActive() const {
        return mRegistry->isAlive(mEntity);
    }

    /**
     * @brief Cismin dünya koordinatlarındaki pozisyonunu döndürür.
     * @return Pozisyon vektörü veya hata kodu.
     */
    [[nodiscard]] std::expected<Vector3, PhysicsError> getPosition() const {
        if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        return mRegistry->getComponent<Pose>(mEntity).position;
    }

    /**
     * @brief Cismin dünya koordinatlarındaki pozisyonunu günceller.
     * @param position Yeni pozisyon vektörü.
     * @return Başarılıysa güncel Body referansı, değilse hata kodu.
     */
    std::expected<Body, PhysicsError> setPosition(const Vector3& position) {
        if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        mRegistry->getComponent<Pose>(mEntity).position = position;
        return *this;
    }

    /**
     * @brief Cismin yönelim (rotation) bilgisini döndürür.
     * @return Yönelim kuaterniyonu veya hata kodu.
     */
    [[nodiscard]] std::expected<Quaternion, PhysicsError> getOrientation() const {
        if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        return mRegistry->getComponent<Pose>(mEntity).orientation;
    }

    /**
     * @brief Cismin doğrusal hızını döndürür.
     * @return Doğrusal hız vektörü (m/s) veya hata kodu.
     */
    [[nodiscard]] std::expected<Vector3, PhysicsError> getLinearVelocity() const {
        if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        return mRegistry->getComponent<Core::Motion>(mEntity).linearVelocity;
    }

    /**
     * @brief Sürekli Çarpışma Algılama (CCD) özelliğini açar/kapatır.
     * @param enabled true: CCD aktif, false: CCD pasif.
     */
    std::expected<Body, PhysicsError> setCCD(bool enabled) {
        if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        mRegistry->getComponent<Core::BodyState>(mEntity).useCCD = enabled;
        return *this;
    }

    /**
     * @brief Cisme anlık bir itme (impulse) kuvveti uygular.
     * @param impulse Uygulanacak itme vektörü (N*s).
     */
    std::expected<Body, PhysicsError> applyLinearImpulse(const Vector3& impulse) {
        if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) || !mRegistry->hasComponent<Core::MassProps>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        auto& motion = mRegistry->getComponent<Core::Motion>(mEntity);
        auto& mass = mRegistry->getComponent<Core::MassProps>(mEntity);
        auto& state = mRegistry->getComponent<Core::BodyState>(mEntity);
        
        motion.linearVelocity += impulse * mass.inverseMass;
        state.isSleeping = false;
        state.sleepTimer = 0.0f;
        return *this;
    }

    /**
     * @brief Cismin doğrusal hızını ayarlar ve uyku modundan çıkarır.
     * @param velocity Yeni doğrusal hız vektörü (m/s).
     */
    std::expected<Body, PhysicsError> setLinearVelocity(const Vector3& velocity) {
        if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) || !mRegistry->hasComponent<Core::BodyState>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        auto& motion = mRegistry->getComponent<Core::Motion>(mEntity);
        motion.linearVelocity = velocity;
        auto& state = mRegistry->getComponent<Core::BodyState>(mEntity);
        state.isSleeping = false;
        state.sleepTimer = 0.0f;
        return *this;
    }

    /**
     * @brief Cismin fiziksel türünü değiştirir.
     * @details Static/Kinematic için hız ve eylemsizlik sıfırlanır. Dynamic için değerler yeniden hesaplanır.
     * @param type Yeni tür (Static, Kinematic veya Dynamic).
     */
    std::expected<Body, PhysicsError> setBodyType(Core::BodyType type) {
        if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity) || !mRegistry->hasComponent<Core::MassProps>(mEntity) || !mRegistry->hasComponent<Core::Motion>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        auto& state = mRegistry->getComponent<Core::BodyState>(mEntity);
        auto& massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
        auto& motion = mRegistry->getComponent<Core::Motion>(mEntity);
        state.type = type;
        
        if (type == Core::BodyType::Static || type == Core::BodyType::Kinematic) {
            massProps.inverseMass = 0.0f; 
            motion.linearVelocity = Vector3(0, 0, 0);
            motion.angularVelocity = Vector3(0, 0, 0);
            massProps.inverseInertiaTensor = Matrix3x3(0,0,0,0,0,0,0,0,0);
            state.isSleeping = false;
            state.sleepTimer = 0.0f;
        } else {
            massProps.inverseMass = (massProps.mass > 0) ? 1.0f / massProps.mass : 1.0f;
            massProps.inverseInertiaTensor = massProps.localInertiaTensor.getInverse();
        }
        return *this;
    }

    /**
     * @brief Cismin kütlesini ayarlar ve ters kütle oranını günceller.
     * @param mass Yeni kütle (kg).
     */
    std::expected<Body, PhysicsError> setMass(float mass) {
        if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity) || !mRegistry->hasComponent<Core::BodyState>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        auto& massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
        auto& state = mRegistry->getComponent<Core::BodyState>(mEntity);
        massProps.mass = mass;
        
        if (state.type == Core::BodyType::Dynamic) {
            massProps.inverseMass = (mass > 0) ? 1.0f / mass : 0.0f;
        }
        return *this;
    }

    /**
     * @brief Cismin yerel eylemsizlik tensörünü ayarlar.
     * @param I Yeni yerel eylemsizlik matrisi (3x3).
     */
    std::expected<Body, PhysicsError> setLocalInertiaTensor(const Matrix3x3& I) {
        if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity)) return std::unexpected(PhysicsError::EntityDead);
        auto& massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
        massProps.localInertiaTensor = I;
        massProps.inverseInertiaTensor = I.getInverse();
        return *this;
    }
};

} // namespace Baryon