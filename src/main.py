"""Viam module entrypoint."""

import asyncio

from viam.module.module import Module

from models.rssi_position_sensor import RssiPositionSensor  # noqa: F401 — registers model

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
