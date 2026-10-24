/*
 * Baryon - A custom physics engine
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "Baryon/Core/Registry.hpp"
#include "Baryon/math/Quaternion.hpp"
#include "Baryon/math/Vector3.hpp"
#include <expected>

/**
 * @file Body.hpp
 * @brief Fiziksel cisim (Body) kullanıcı API'si.
 * ECS varlıklarını ve bileşenlerini güvenli bir şekilde yönetmek için
 * kullanılır.
 */
#include "../Core/PhysicsError.hpp"

namespace Baryon {


/**
 * @class Body
 * @brief Fiziksel cisimlere güvenli erişim sağlayan sarmalayıcı sınıf.
 * ECS kayıt defteri üzerinden veri erişimi ve std::expected ile hata yönetimi
 * sağlar.
 */
class Body {
private:
  ecs::Entity mEntity;       ///< Sarmalanan varlığın kimliği.
  Core::Registry *mRegistry; ///< ECS kayıt defterine işaretçi.

public:
  /**
   * @brief Body sarmalayıcısı oluşturur.
   * @param entity Sarmalanacak varlık.
   * @param registry ECS kayıt defteri.
   */
  Body(ecs::Entity entity, Core::Registry &registry)
      : mEntity(entity), mRegistry(&registry) {}

  /** @brief Sarmalanan varlığın kimliğini (Entity ID) döndürür. */
  [[nodiscard]] ecs::Entity getEntity() const { return mEntity; }

  /** @brief Varlığın sistemde hala aktif olup olmadığını kontrol eder. */
  [[nodiscard]] bool isActive() const { return mRegistry->isAlive(mEntity); }

  /**
   * @brief Cismin dünya koordinatlarındaki pozisyonunu döndürür.
   * @return Pozisyon vektörü veya hata kodu.
   */
  [[nodiscard]] std::expected<Vector3, PhysicsError> getPosition() const {
    if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Pose>(mEntity).position;
  }

  /**
   * @brief Cismin dünya koordinatlarındaki pozisyonunu günceller.
   * @param position Yeni pozisyon vektörü.
   * @return Başarılıysa güncel Body referansı, değilse hata kodu.
   */
  std::expected<Body, PhysicsError> setPosition(const Vector3 &position) {
    if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<Pose>(mEntity).position = position;
    return *this;
  }

  /**
   * @brief Cismin yönelim (rotation) bilgisini döndürür.
   * @return Yönelim kuaterniyonu veya hata kodu.
   */
  [[nodiscard]] std::expected<Quaternion, PhysicsError> getOrientation() const {
    if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Pose>(mEntity).orientation;
  }

  /**
   * @brief Cismin doğrusal hızını döndürür.
   * @return Doğrusal hız vektörü (m/s) veya hata kodu.
   */
  [[nodiscard]] std::expected<Vector3, PhysicsError> getLinearVelocity() const {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::Motion>(mEntity).linearVelocity;
  }

  /**
   * @brief Sürekli Çarpışma Algılama (CCD) özelliğini açar/kapatır.
   * @param enabled true: CCD aktif, false: CCD pasif.
   */
  std::expected<Body, PhysicsError> setCCD(bool enabled) {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<Core::BodyState>(mEntity).useCCD = enabled;
    return *this;
  }

  /**
   * @brief Spekülatif Kontak (öngörülü çarpışma) özelliğini açar/kapatır.
   * @param enabled true: spekülatif kontak aktif, false: pasif.
   */
  std::expected<Body, PhysicsError> setSpeculativeContacts(bool enabled) {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<Core::BodyState>(mEntity).useSpeculativeContacts = enabled;
    return *this;
  }

  /**
   * @brief Cismin spekülatif kontak özelliğinin aktif olup olmadığını döndürür.
   */
  [[nodiscard]] std::expected<bool, PhysicsError> hasSpeculativeContacts() const {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::BodyState>(mEntity).useSpeculativeContacts;
  }

  /**
   * @brief Cisme anlık bir itme (impulse) kuvveti uygular.
   * @param impulse Uygulanacak itme vektörü (N*s).
   */
  std::expected<Body, PhysicsError> applyLinearImpulse(const Vector3 &impulse) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::MassProps>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    auto &mass = mRegistry->getComponent<Core::MassProps>(mEntity);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);

    motion.linearVelocity += impulse * mass.inverseMass;
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  /**
   * @brief Cismin doğrusal hızını ayarlar ve uyku modundan çıkarır.
   * @param velocity Yeni doğrusal hız vektörü (m/s).
   */
  std::expected<Body, PhysicsError> setLinearVelocity(const Vector3 &velocity) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    motion.linearVelocity = velocity;
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  /**
   * @brief Cismin fiziksel türünü değiştirir.
   * @details Static/Kinematic için hız ve eylemsizlik sıfırlanır. Dynamic için
   * değerler yeniden hesaplanır.
   * @param type Yeni tür (Static, Kinematic veya Dynamic).
   */
  std::expected<Body, PhysicsError> setBodyType(Core::BodyType type) {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity) ||
        !mRegistry->hasComponent<Core::MassProps>(mEntity) ||
        !mRegistry->hasComponent<Core::Motion>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);
    auto &massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    state.type = type;

    if (type == Core::BodyType::Static || type == Core::BodyType::Kinematic) {
      massProps.inverseMass = 0.0f;
      motion.linearVelocity = Vector3(0, 0, 0);
      motion.angularVelocity = Vector3(0, 0, 0);
      motion.splitLinearVelocity = Vector3(0, 0, 0);
      motion.splitAngularVelocity = Vector3(0, 0, 0);
      massProps.inverseInertiaTensor = Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
      state.isSleeping = false;
      state.sleepTimer = 0.0f;
    } else {
      massProps.inverseMass =
          (massProps.mass > 0) ? 1.0f / massProps.mass : 1.0f;
      massProps.inverseInertiaTensor = massProps.inverseLocalInertiaTensor;
    }
    return *this;
  }

  /**
   * @brief Cismin kütlesini ayarlar ve ters kütle ile eylemsizlik tensörünü ölçekler.
   * @param mass Yeni kütle (kg).
   */
  [[nodiscard]] std::expected<Body, PhysicsError> setMass(float mass) {
    if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    if (mass <= 0.0f)
      return std::unexpected(PhysicsError::InvalidValue);
    auto &massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);
    float oldMass = massProps.mass;
    massProps.mass = mass;
    state.isSleeping = false;
    state.sleepTimer = 0.0f;

    if (state.type == Core::BodyType::Dynamic) {
      massProps.inverseMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    }

    // Eylemsizlik tensörünü yeni/eski kütle oranında ölçekle
    if (oldMass > 0.0f && mass > 0.0f) {
      float scale = mass / oldMass;
      massProps.localInertiaTensor = massProps.localInertiaTensor * scale;
      massProps.inverseLocalInertiaTensor = massProps.localInertiaTensor.getInverse();
      if (state.type == Core::BodyType::Dynamic) {
        massProps.inverseInertiaTensor = massProps.inverseLocalInertiaTensor;
      }
    }
    return *this;
  }

  /**
   * @brief Cismin yerel eylemsizlik tensörünü ayarlar.
   * @param I Yeni yerel eylemsizlik matrisi (3x3).
   */
  [[nodiscard]] std::expected<Body, PhysicsError> setLocalInertiaTensor(const Matrix3x3 &I) {
    if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &massProps = mRegistry->getComponent<Core::MassProps>(mEntity);
    massProps.localInertiaTensor = I;
    massProps.inverseLocalInertiaTensor = I.getInverse();
    massProps.inverseInertiaTensor = massProps.inverseLocalInertiaTensor;
    return *this;
  }

  /**
   * @brief Cismin kütlesini döndürür.
   */
  [[nodiscard]] std::expected<float, PhysicsError> getMass() const {
    if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::MassProps>(mEntity).mass;
  }

  /**
   * @brief Cismin yerel eylemsizlik tensörünü döndürür.
   */
  [[nodiscard]] std::expected<Matrix3x3, PhysicsError> getLocalInertiaTensor() const {
    if (!isActive() || !mRegistry->hasComponent<Core::MassProps>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::MassProps>(mEntity).localInertiaTensor;
  }

  std::expected<Body, PhysicsError> applyForce(const Vector3 &force) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);

    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);

    motion.externalForce += force;
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  std::expected<Body, PhysicsError> applyTorque(const Vector3 &torque) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);

    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);

    motion.externalTorque += torque;
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  std::expected<Body, PhysicsError>
  applyAngularImpulse(const Vector3 &angularImpulse) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::MassProps>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);

    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    auto &mass = mRegistry->getComponent<Core::MassProps>(mEntity);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);

    motion.angularVelocity += mass.inverseInertiaTensor * angularImpulse;
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  [[nodiscard]] std::expected<Vector3, PhysicsError>
  getAngularVelocity() const {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::Motion>(mEntity).angularVelocity;
  }

  std::expected<Body, PhysicsError> setAngularVelocity(const Vector3 &angVel) {
    if (!isActive() || !mRegistry->hasComponent<Core::Motion>(mEntity) ||
        !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);

    auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
    motion.angularVelocity = angVel;
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  std::expected<Body, PhysicsError>
  setOrientation(const Quaternion &orientation) {
    if (!isActive() || !mRegistry->hasComponent<Pose>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<Pose>(mEntity).orientation = orientation;
    return *this;
  }

  std::expected<Body, PhysicsError> setMaterial(float friction,
                                                float restitution) {
    if (!isActive() || !mRegistry->hasComponent<Core::Material>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &mat = mRegistry->getComponent<Core::Material>(mEntity);
    mat.friction = std::max(0.0f, friction);
    mat.restitution = std::clamp(restitution, 0.0f, 1.0f);
    return *this;
  }

  [[nodiscard]] std::expected<bool, PhysicsError> isSleeping() const {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::BodyState>(mEntity).isSleeping;
  }

  std::expected<Body, PhysicsError> setTrigger(bool isTrigger) {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<ecs::ColliderData>(mEntity).isTrigger = isTrigger;
    return *this;
  }

  [[nodiscard]] std::expected<bool, PhysicsError> isTrigger() const {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<ecs::ColliderData>(mEntity).isTrigger;
  }

  std::expected<Body, PhysicsError> setCollisionLayer(uint32_t layer) {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<ecs::ColliderData>(mEntity).layer = layer;
    return *this;
  }

  [[nodiscard]] std::expected<uint32_t, PhysicsError> getCollisionLayer() const {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<ecs::ColliderData>(mEntity).layer;
  }

  std::expected<Body, PhysicsError> setCollisionMask(uint32_t mask) {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    mRegistry->getComponent<ecs::ColliderData>(mEntity).mask = mask;
    return *this;
  }

  [[nodiscard]] std::expected<uint32_t, PhysicsError> getCollisionMask() const {
    if (!isActive() || !mRegistry->hasComponent<ecs::ColliderData>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<ecs::ColliderData>(mEntity).mask;
  }

  /**
   * @brief Cismin kilitli serbestlik derecelerini (DOF Lock) ayarlar.
   * @param axes Kilitlenecek eksenler bitmask'i (LockAxis kombinasyonu).
   * @return Başarılıysa güncel Body referansı, değilse hata kodu.
   */
  std::expected<Body, PhysicsError> setLockedAxes(uint8_t axes) {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    auto &state = mRegistry->getComponent<Core::BodyState>(mEntity);
    state.lockedAxes = axes;

    // Kilitlenen eksenlerdeki hızları sıfırla
    if (mRegistry->hasComponent<Core::Motion>(mEntity)) {
      auto &motion = mRegistry->getComponent<Core::Motion>(mEntity);
      if (axes & Core::LockAxis::LinearX) { motion.linearVelocity.x = 0.0f; motion.splitLinearVelocity.x = 0.0f; }
      if (axes & Core::LockAxis::LinearY) { motion.linearVelocity.y = 0.0f; motion.splitLinearVelocity.y = 0.0f; }
      if (axes & Core::LockAxis::LinearZ) { motion.linearVelocity.z = 0.0f; motion.splitLinearVelocity.z = 0.0f; }
      if (axes & Core::LockAxis::AngularX) { motion.angularVelocity.x = 0.0f; motion.splitAngularVelocity.x = 0.0f; }
      if (axes & Core::LockAxis::AngularY) { motion.angularVelocity.y = 0.0f; motion.splitAngularVelocity.y = 0.0f; }
      if (axes & Core::LockAxis::AngularZ) { motion.angularVelocity.z = 0.0f; motion.splitAngularVelocity.z = 0.0f; }
    }
    state.isSleeping = false;
    state.sleepTimer = 0.0f;
    return *this;
  }

  /**
   * @brief Cismin kilitli eksen bitmask'ini döndürür.
   * @return Kilitli eksenler veya hata kodu.
   */
  [[nodiscard]] std::expected<uint8_t, PhysicsError> getLockedAxes() const {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return std::unexpected(PhysicsError::EntityDead);
    return mRegistry->getComponent<Core::BodyState>(mEntity).lockedAxes;
  }

  /**
   * @brief Belirli bir eksenin kilitli olup olmadığını sorgular.
   * @param axis Sorgulanacak eksen.
   * @return Eksen kilitliyse true, serbestse veya cisim geçersizse false.
   */
  [[nodiscard]] bool isAxisLocked(Core::LockAxis axis) const {
    if (!isActive() || !mRegistry->hasComponent<Core::BodyState>(mEntity))
      return false;
    return (mRegistry->getComponent<Core::BodyState>(mEntity).lockedAxes & static_cast<uint8_t>(axis)) != 0;
  }
};
} // namespace Baryon
