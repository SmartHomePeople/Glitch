import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from simulator import scenarios as S  # noqa: E402


@pytest.fixture(scope="session")
def firmware():
    """AutoPark.cpp under test: $GLITCH_FIRMWARE or RobotFirmware/opmodes/AutoPark.cpp."""
    return S.default_firmware()
