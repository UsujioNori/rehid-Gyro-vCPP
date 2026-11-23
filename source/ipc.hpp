#pragma once
#include <3ds.h>
#include "hid.hpp"

struct IRCommandDiagnostic {
    uint64_t tick;
    uint32_t command;
    uint32_t detail;
    uint32_t result;
};

extern volatile IRCommandDiagnostic diag_ir_command_history[16];
extern volatile uint32_t diag_ir_command_history_next;
extern volatile uint32_t diag_ir_command_history_count;

namespace IPC {
    void HandleHidCommands(Hid *hid);
    void HandleMockIRUserCommands(Hid *hid);
};