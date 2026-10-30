// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);
// Remembers the UI language on its own (Linux keeps it outside the calibration profile).
void SaveLanguage(CalibrationContext &ctx);
