#include "Pad.hpp"
#include <atomic>

std::atomic<uint32_t> diag_pad_raw_seen {0};
std::atomic<uint32_t> diag_pad_ring_seen {0};
std::atomic<uint32_t> diag_pad_pressed_seen {0};
extern "C" {
#include "services/gpio.h"
}

void Pad::Initialize() {
    if (!m_isinitialized) {
        m_isinitialized = true;
        m_latestkeys = (vu32)(IOHIDPAD) ^ 0xFFF;
        m_dummy = 0x4001;
        m_slider.GetConfigSettings();
        m_circlepad.GetConfigSettings();
        svcCreateTimer(&m_timer, RESET_ONESHOT);
        svcCreateEvent(&m_event, RESET_ONESHOT);
    }
}

void Pad::SetTimer() {
    if (R_FAILED(svcSetTimer(m_timer, 4000000LL, 0LL)))
        svcBreak(USERBREAK_ASSERT);
}

extern u32 debugpadkeys;
extern CirclePadEntry debugpadstick;
void Pad::ReadFromIO(PadEntry *entry, uint32_t *raw, CirclePadEntry *circlepad, Remapper *remapper) {
    volatile uint32_t latest = (vu32)(IOHIDPAD) ^ 0xFFF;

    uint32_t val = 0;
    GPIOHID_GetData(0x4001, &val);
    *raw = latest;
    latest = latest & ~(2 * (latest & 0x40) | ((latest & 0x20u) >> 1));

#ifdef ENABLE_DEBUGPAD_REMAPPING

    if (val & 0x1) {
        latest |= debugpadkeys;
        circlepad->x = debugpadstick.x;
        circlepad->y = debugpadstick.y;
    }

#endif

    m_ir->ScanInput(remapper);
    CPPEntry cppentry = m_ir->GetInputs();
    uint32_t additionalcpad = CirclePad::ConvertToHidButtons<CirclePadMode::CPAD>(circlepad, latest); // if need be this also sets the circlepad entry to 0

    latest = latest | cppentry.currpadstate | remapper->m_remaptouchkeys | additionalcpad;
    latest = remapper->Remap(latest, circlepad, &cppentry.circlepadstate);

    entry->pressedpadstate = (latest ^ m_latestkeys) & ~m_latestkeys;
    entry->releasedpadstate = (latest ^ m_latestkeys) & m_latestkeys;
    entry->currpadstate = latest;
    m_latestkeys = latest;
}

void Pad::Sampling(u32 rcpr, Remapper *remapper) {
    PadEntry finalentry;
    uint32_t latest;
    static float sliderval = 0.0f;
    CirclePadEntry rawcirclepad;
    rawcirclepad.x = rcpr & 0xFFF;
    rawcirclepad.y = (rcpr & 0xFFF000) >> 12;
    CirclePadEntry finalcirclepad {0, 0};
    svcSetTimer(m_timer, 4000000LL, 0LL);

    // Many people use rehid to just disable cpad, so make it simpler
    // to build.
#ifndef DISABLE_CPAD
    m_circlepad.RawToCirclePadCoords(&finalcirclepad, rawcirclepad);

    /* These raw values can then be later on accessed during cpadtocnub
       remapping.
    */
    remapper->SaveRawCpad(&rawcirclepad);
#endif

    if (m_counter % 3 == 0) {
        m_slider.ReadValuesFromMCU();
        sliderval = m_slider.Normalize();
        *(float*)0x1FF81080 = sliderval;
    }

    ++m_counter;
    ReadFromIO(&finalentry, &latest, &finalcirclepad, remapper);
#ifdef TEST_EXTEND_PAD_PRESSES
    // Expose each selected press edge for up to 16 samples while held.
    constexpr uint32_t testButtons = KEY_A | KEY_B | KEY_X | KEY_Y |
        KEY_DUP | KEY_DDOWN | KEY_DLEFT | KEY_DRIGHT;
    static uint32_t recentPresses[16] {};
    recentPresses[m_counter % 16] = finalentry.pressedpadstate & testButtons;
    uint32_t extendedPresses = 0;
    for (uint32_t presses : recentPresses)
        extendedPresses |= presses;
    finalentry.pressedpadstate |= extendedPresses & finalentry.currpadstate;
#endif
    m_ring->SetCurrPadState(latest, rawcirclepad);
    m_ring->Set3dSliderVal(sliderval);
    m_ring->WriteToRing(&finalentry, &finalcirclepad);
    if (latest)
        diag_pad_raw_seen.fetch_or(latest, std::memory_order_relaxed);
    if (finalentry.currpadstate)
        diag_pad_ring_seen.fetch_or(finalentry.currpadstate, std::memory_order_relaxed);
    if (finalentry.pressedpadstate)
        diag_pad_pressed_seen.fetch_or(finalentry.pressedpadstate, std::memory_order_relaxed);
    svcSignalEvent(m_event);
}
