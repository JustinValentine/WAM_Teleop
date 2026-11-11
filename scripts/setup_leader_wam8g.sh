#!/bin/bash
# The wam8g (leader_wam8g, dynamics_excitation_wam8g) uses the robot's own
# configuration, config/wam8g at the top of the workspace repository, which
# holds its calibration. Set bus.port there to the wam8g's CAN interface.

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export BARRETT_CONFIG_FILE="$(cd "$PACKAGE_DIR/../../../config/wam8g" && pwd)/default.conf"
