import importlib

import pytest


def test_server_module_imports():
    pytest.importorskip("rclpy")
    mod = importlib.import_module("togsim_hmi.server")
    assert hasattr(mod, "HmiBridge") and hasattr(mod, "build_app")
