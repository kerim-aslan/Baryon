#include "TestCommon.hpp"
#include <Baryon/Core/DebugManager.hpp>
#include <iostream>

using namespace Baryon::Debug;

int main() {
    std::cout << "=== V8: DebugManager Devre Disi Degerlendirme ve iostream Testi ===" << std::endl;

    auto& dbg = DebugManager::get();

    // 1. Kategori kapalıyken yan etkili argüman hesaplanmamalı
    {
        dbg.setGlobalEnable(true);
        dbg.setCategory(LogCategory::COLLISION_NARROW, false);

        int sideEffectCounter = 0;
        auto expensiveFn = [&]() {
            sideEffectCounter++;
            return 42;
        };

        BARYON_LOG_NARROW("Test narrow log with value: ", expensiveFn());

        CHECK_MSG(sideEffectCounter == 0,
                  "Kategori kapaliyken log argumani hesaplanmamali (sideEffectCounter == 0 olmali)");
    }

    // 2. Global devre dışıyken yan etkili argüman hesaplanmamalı
    {
        dbg.setGlobalEnable(false);
        dbg.setCategory(LogCategory::GENERAL, true);

        int sideEffectCounter = 0;
        auto expensiveFn = [&]() {
            sideEffectCounter++;
            return 99;
        };

        BARYON_LOG_INFO("Test info log with value: ", expensiveFn());

        CHECK_MSG(sideEffectCounter == 0,
                  "GlobalEnable false iken log argumani hesaplanmamali (sideEffectCounter == 0 olmali)");
    }

    // 3. Kategori açıkken yan etkili argüman hesaplanmalı
    {
        dbg.setGlobalEnable(true);
        dbg.setCategory(LogCategory::COLLISION_NARROW, true);

        int sideEffectCounter = 0;
        auto expensiveFn = [&]() {
            sideEffectCounter++;
            return 123;
        };

        BARYON_LOG_NARROW("Test narrow log with value: ", expensiveFn());

        CHECK_MSG(sideEffectCounter == 1,
                  "Kategori acikken log argumani 1 kez hesaplanmali (sideEffectCounter == 1 olmali)");

        // Geri kapat
        dbg.setCategory(LogCategory::COLLISION_NARROW, false);
    }

    // 4. Farklı log makrolarının kapalı durum kontrolleri
    {
        dbg.setGlobalEnable(true);
        dbg.setCategory(LogCategory::SOLVER_IMPULSE, false);
        dbg.setCategory(LogCategory::SOLVER_POSITION, false);
        dbg.setCategory(LogCategory::ISLAND_SLEEP, false);

        int impulseCounter = 0;
        int posCounter = 0;
        int sleepCounter = 0;

        BARYON_LOG_IMPULSE("Impulse: ", (++impulseCounter));
        BARYON_LOG_POS("Pos: ", (++posCounter));
        BARYON_LOG_SLEEP("Sleep: ", (++sleepCounter));

        CHECK_MSG(impulseCounter == 0, "BARYON_LOG_IMPULSE kapaliyken arguman isletilmemeli");
        CHECK_MSG(posCounter == 0, "BARYON_LOG_POS kapaliyken arguman isletilmemeli");
        CHECK_MSG(sleepCounter == 0, "BARYON_LOG_SLEEP kapaliyken arguman isletilmemeli");
    }

    return reportAndExit();
}
