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

/**
 * @file Simulator.cpp
 * @brief Simulator sınıfının uygulama (implementation) dosyası.
 * @details Fizik simülasyonunun ana döngüsünü, varlık yönetimini ve
 *          dünya sorgularını (raycast) koordine eden merkezi mantığı içerir.
 */

#include "Baryon/Simulator.hpp"

#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/collision/NarrowPhase.hpp"

#include <iostream>
#include <ranges>

namespace Baryon {

/**
 * @brief Simulator kurucusu. Tüm fizik sistemlerini hiyerarşik sırayla başlatır.
 * @details Sistemler arasındaki bağımlılıklar nedeniyle başlatma sırası önemlidir:
 *          Bellek yöneticisi ve kayıt defteri (Registry) tüm sistemlerin temelidir.
 */
Simulator::Simulator() : mRegistry(mMemoryManager),
						 mIntegrator(mRegistry),
						 mHierarchySystem(mRegistry),
						 mSpatialPartitioning(mRegistry, mMemoryManager),
						 mCollisionSystem(mRegistry, mSpatialPartitioning),
						 mSolver(mRegistry),
						 mIslandSystem(mRegistry) {
	setSettings(mSettings);
}

void Simulator::setSettings(const SimulationSettings &settings) {
	mSettings = settings;
	mFixedTimeStep = settings.fixedTimeStep;
	mMaxSubSteps = settings.maxSubSteps;
	mIslandSystem.setSleepThresholds(settings.sleepLinearThreshold, settings.sleepAngularThreshold, settings.timeToSleep);
	mSpatialPartitioning.setVelocityExpansion(settings.broadPhaseVelocityExpansion);
}

/**
 * @brief Fizik dünyasında yeni bir cisim (Body) oluşturur.
 * @details İşlem sırası:
 *          1. ECS üzerinde yeni bir varlık (Entity) yaratılır.
 *          2. Hareket, kütle ve çarpışma verilerini tutan bileşenler atanır.
 *          3. Geniş faz (AABB Ağacı) kaydı yapılır.
 *          4. Cismin geometrisine göre eylemsizlik tensörü (inertia tensor) otomatik hesaplanır.
 *             Hesaplamada şu formül temel alınır:
 *             $$I_{xx} = \frac{m}{12}(h^2 + d^2), \quad I_{yy} = \frac{m}{12}(w^2 + d^2), \quad I_{zz} = \frac{m}{12}(w^2 + h^2)$$
 *
 * @param transform Cismin başlangıç konumu ve yönelimi.
 * @param shape Cismin çarpışma geometrisi.
 * @return Oluşturulan cismi yönetmek için kullanılan Body nesnesi.
 */
Body Simulator::createBody(const Pose &transform, const collision::CollisionShape &shape) {
	auto entityRes = mRegistry.createEntity();
	if (!entityRes) {
		return Body(ecs::Entity{0xFFFFFFFFFFFFFFFFULL}, mRegistry);
	}
	ecs::Entity entity = *entityRes;

	mRegistry.addComponent(entity, transform);

	mRegistry.addComponent(entity, ecs::ColliderData{ shape });

	// Temel fizik bileşenlerinin ilk değerlerle atanması
	mRegistry.addComponent(entity, Core::Motion{});
	mRegistry.addComponent(entity, Core::MassProps{});
	mRegistry.addComponent(entity, Core::Material{});
	mRegistry.addComponent(entity, Core::BodyState{});

	// Performans için uzaysal bölümleme ağacına ekleme
	mSpatialPartitioning.addEntityToTree(entity);

	Body rb(entity, mRegistry);

	// Kütle ve eylemsizlik tensörü hesaplamaları (varsayılan: 1kg)
	(void)rb.setMass(1.0f);

	Matrix3x3 I = Matrix3x3::identity();
	std::visit([&](const auto &s) {
		using T = std::decay_t<decltype(s)>;
		if constexpr (std::is_same_v<T, collision::SphereShape>) {
			float val = 0.4f * 1.0f * s.radius * s.radius;
			I[0].x = val;
			I[1].y = val;
			I[2].z = val;
		} else if constexpr (std::is_same_v<T, collision::BoxShape>) {
			Vector3 size = s.halfExtents * 2.0f;
			I[0].x = (1.0f / 12.0f) * 1.0f * (size.y * size.y + size.z * size.z);
			I[1].y = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.z * size.z);
			I[2].z = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.y * size.y);
		} else if constexpr (std::is_same_v<T, collision::CapsuleShape>) {
			float r = s.radius;
			float h = s.height;
			// Silindir + yarımküre yaklaşımı
			float cylinderMass = (h / (h + (4.0f / 3.0f) * r));
			float capMass = 1.0f - cylinderMass;
			float ix = cylinderMass * ((1.0f / 12.0f) * h * h + (1.0f / 4.0f) * r * r) +
					capMass * ((2.0f / 5.0f) * r * r + (3.0f / 8.0f) * h * r + (1.0f / 4.0f) * h * h);
			float iy = (1.0f / 2.0f) * cylinderMass * r * r + (2.0f / 5.0f) * capMass * r * r;
			I[0].x = ix;
			I[1].y = iy;
			I[2].z = ix;
		} else if constexpr (std::is_same_v<T, collision::CompoundShape>) {
			I = s.computeInertiaTensor(1.0f);
		} else {
			collision::AABB localAABB = shape.computeLocalAABB();
			Vector3 size = localAABB.maxBounds - localAABB.minBounds;
			I[0].x = (1.0f / 12.0f) * 1.0f * (size.y * size.y + size.z * size.z);
			I[1].y = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.z * size.z);
			I[2].z = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.y * size.y);
		}
	},
			shape.getVariant());
	(void)rb.setLocalInertiaTensor(I);

	return rb;
}

/**
 * @brief Bir cismi fizik dünyasından ve bellekten tamamen siler.
 * @param body Silinecek cisim.
 */
void Simulator::destroyBody(Body &body) {
	ecs::Entity entity = body.getEntity();
	if (mRegistry.isAlive(entity)) {
		// 1. Ağaç üzerinden bu cisimle temas/kesişim halindeki uyuyan cisimleri uyandır
		if (mRegistry.hasComponent<ecs::ColliderData>(entity)) {
			auto &collider = mRegistry.getComponent<ecs::ColliderData>(entity);
			if (collider.treeNodeIndex != collision::NULL_NODE) {
				collision::AABB aabb = mSpatialPartitioning.getTree().getNodeAABB(collider.treeNodeIndex);
				const float expand = 0.05f;
				aabb.minBounds -= Vector3(expand, expand, expand);
				aabb.maxBounds += Vector3(expand, expand, expand);
				mSpatialPartitioning.getTree().query(aabb, [&](ecs::Entity other) {
					if (other.id != entity.id && mRegistry.isAlive(other) && mRegistry.hasComponent<Core::BodyState>(other)) {
						auto &st = mRegistry.getComponent<Core::BodyState>(other);
						st.isSleeping = false;
						st.sleepTimer = 0.0f;
					}
				});
			}
		}

		// 2. Aktif temas manifoldları üzerinden de kontrol et
		for (const auto &[key, manifold] : mCollisionSystem.getManifolds()) {
			if (manifold.entityA == entity) {
				if (mRegistry.isAlive(manifold.entityB) && mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
					auto &st = mRegistry.getComponent<Core::BodyState>(manifold.entityB);
					st.isSleeping = false;
					st.sleepTimer = 0.0f;
				}
			} else if (manifold.entityB == entity) {
				if (mRegistry.isAlive(manifold.entityA) && mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
					auto &st = mRegistry.getComponent<Core::BodyState>(manifold.entityA);
					st.isSleeping = false;
					st.sleepTimer = 0.0f;
				}
			}
		}
		auto &distConstraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
		for (const auto &c : distConstraints) {
			if (c.entityA == entity && mRegistry.isAlive(c.entityB) && mRegistry.hasComponent<Core::BodyState>(c.entityB)) {
				auto &st = mRegistry.getComponent<Core::BodyState>(c.entityB);
				st.isSleeping = false;
				st.sleepTimer = 0.0f;
			} else if (c.entityB == entity && mRegistry.isAlive(c.entityA) && mRegistry.hasComponent<Core::BodyState>(c.entityA)) {
				auto &st = mRegistry.getComponent<Core::BodyState>(c.entityA);
				st.isSleeping = false;
				st.sleepTimer = 0.0f;
			}
		}
		auto &revConstraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
		for (const auto &c : revConstraints) {
			if (c.entityA == entity && mRegistry.isAlive(c.entityB) && mRegistry.hasComponent<Core::BodyState>(c.entityB)) {
				auto &st = mRegistry.getComponent<Core::BodyState>(c.entityB);
				st.isSleeping = false;
				st.sleepTimer = 0.0f;
			} else if (c.entityB == entity && mRegistry.isAlive(c.entityA) && mRegistry.hasComponent<Core::BodyState>(c.entityA)) {
				auto &st = mRegistry.getComponent<Core::BodyState>(c.entityA);
				st.isSleeping = false;
				st.sleepTimer = 0.0f;
			}
		}

		mSpatialPartitioning.removeEntityFromTree(entity);
		mRegistry.destroyEntity(entity);
	}
}

/**
 * @brief Fizik simülasyonunu ana döngü içerisinde ilerletir.
 * @details Sabit zaman adımı (fixed timestep) ve biriktirici (accumulator) mantığı kullanılır.
 *          Bu sayede FPS dalgalanmalarından bağımsız, deterministik bir fizik dünyası sağlanır.
 *
 * Fiziksel İşlem Akışı:
 * 1. Hız Entegrasyonu (Kuvvet -> Hız).
 * 2. CCD İşlemleri (Tünelleme önleyici).
 * 3. Geniş Faz (Broad Phase) Güncellemesi.
 * 4. Dar Faz (Narrow Phase) ve Temas Üretimi.
 * 5. Ada Sistemi (Island) Aktif Adaların Kurulması.
 * 6. Kısıtlama Çözücü (Constraint Solver).
 * 7. Konum Entegrasyonu (Hız -> Konum).
 * 8. Uyku Yönetimi (Çözücü sonrası hızlara göre).
 * 9. Hiyerarşi Güncellemesi.
 */
void Simulator::step(float deltaTime) {
	if (deltaTime <= 0.0f) {
		return;
	}

	mAccumulator += deltaTime;

	// Ölüm Spirali (Spiral of Death) Koruması:
	// Tek bir karede en fazla mMaxSubSteps alt adım çalıştırılır. Aşırı gecikmeler birikip sonraki kareleri kitlemez.
	int subStepsLeft = mMaxSubSteps;
	while (mAccumulator >= mFixedTimeStep && subStepsLeft > 0) {
		float dt = mFixedTimeStep;
		subStepsLeft--;

		// 1. Dinamik kuvvetleri hıza aktar
		mIntegrator.integrateVelocities(dt);

		// 1b. Hızlı hareket eden nesneler için sürekli çarpışma kontrolü
		{
			auto &statePool = mRegistry.getComponentPool<Core::BodyState>();
			auto &states = statePool.getAllData();
			auto &entities = statePool.getAllEntities();
			for (auto &&[entity, state] : std::views::zip(entities, states)) {
				if (state.type != Core::BodyType::Dynamic || !state.useCCD || state.isSleeping) {
					continue;
				}
				mCollisionSystem.applyCCD(entity, dt);
			}
		}

		// 2. Sınır kutularını (AABB) güncelle ve adayları filtrele
		mSpatialPartitioning.step();

		// 3. Kesin çarpışma testlerini yap ve temas noktalarını üret
		mCollisionSystem.step();

		// 4. Etkileşim halindeki nesne gruplarını ayır (aktif adaları kur)
		auto islands = mIslandSystem.buildIslands(mCollisionSystem.getManifolds());

		// 5. Her nesne grubu için fiziksel tepkileri hesapla
		mSolver.executeIslands(islands, mCollisionSystem.getManifolds(), dt, mSettings.velocityIterations, mSettings.positionIterations);

		// 6. Güncellenmiş hızları kullanarak konumları değiştir
		mIntegrator.integratePositions(dt);

		// 6b. Uyku durumlarını çözücü ve konum güncellemesi sonrası hızlara göre güncelle
		if (mSettings.enableSleeping) {
			mIslandSystem.updateSleep(islands, dt);
		}

		// 7. Bağlı nesne hiyerarşilerini (ebeveyn-çocuk) uyarla
		mHierarchySystem.step();

		// 8. O kareye özel kullanılan geçici bellek alanlarını temizle
		mMemoryManager.resetSingleFrameAllocator();

		mAccumulator -= mFixedTimeStep;
	}

	// Eğer kare çok uzadıysa birikintiyi sıfırla (gecikme zincirleme devam etmesin)
	if (subStepsLeft == 0) {
		mAccumulator = 0.0f;
	}
}

/**
 * @brief Sahne genelindeki yerçekimi veya benzeri ivme alanını ayarlar.
 */
void Simulator::setAccelerationField(const Vector3 &accelerationField) {
	mIntegrator.setAccelerationField(accelerationField);
	mCollisionSystem.setAccelerationField(accelerationField);
}

/**
 * @brief Dünya üzerinde ışın fırlatarak en yakın objeyi tespit eder.
 * @details Performans için önce AABB ağacıyla kaba bir eleme yapılır,
 *          ardından adaylar üzerinde hassas geometrik test uygulanır.
 *
 * @param origin Işın başlangıç noktası.
 * @param direction Işın yönü.
 * @param maxDist Maksimum menzil.
 * @return Çarpışma detaylarını içeren RaycastHit yapısı.
 */
collision::RaycastHit Simulator::raycast(const Vector3 &origin, const Vector3 &direction, float maxDist) {
	collision::RaycastHit closestHit;
	closestHit.hasHit = false;
	closestHit.distance = maxDist;

	collision::Ray ray(origin, direction, maxDist);

	mSpatialPartitioning.getTree().query(ray.computeAABB(), [&](ecs::Entity obstacle) {
		if (!mRegistry.hasComponent<Pose>(obstacle) || !mRegistry.hasComponent<ecs::ColliderData>(obstacle)) {
			return;
		}

		const auto &obsTrans = mRegistry.getComponent<Pose>(obstacle);
		const auto &obsColl = mRegistry.getComponent<ecs::ColliderData>(obstacle);

		collision::RaycastHit hit;
		if (collision::NarrowPhase::raycast(obsColl.shape, obsTrans, ray, hit)) {
			if (hit.distance < closestHit.distance) {
				closestHit = hit;
				closestHit.entity = obstacle;
			}
		}
	});

	return closestHit;
}

/**
 * @brief Belirli bir nesneye ait tüm aktif temas noktalarını listeler.
 */
std::vector<collision::ContactManifold> Simulator::getContactsForEntity(ecs::Entity entity) const {
	std::vector<collision::ContactManifold> result;
	const auto &manifolds = mCollisionSystem.getManifolds();
	for (const auto &[key, manifold] : manifolds) {
		if (manifold.entityA.id == entity.id || manifold.entityB.id == entity.id) {
			result.push_back(manifold);
		}
	}
	return result;
}

void Simulator::printAABBs() {
	std::cout << "\n--- Baryon Physics AABB Dump ---\n";
	const auto &colliderPool = mRegistry.getComponentPool<ecs::ColliderData>();
	const auto &entities = colliderPool.getAllEntities();
	for (auto entity : entities) {
		if (!mRegistry.hasComponent<Pose>(entity)) {
			continue;
		}
		const auto &collider = mRegistry.getComponent<ecs::ColliderData>(entity);
		const auto &pose = mRegistry.getComponent<Pose>(entity);

		collision::AABB localAABB = collider.shape.computeLocalAABB();
		Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

		std::cout << "Entity " << entity.id << " Pos: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
				  << "Local AABB Extents: [" << extents.x << ", " << extents.y << ", " << extents.z << "]\n";
	}
	std::cout << "--------------------------------\n";
}

} // namespace Baryon