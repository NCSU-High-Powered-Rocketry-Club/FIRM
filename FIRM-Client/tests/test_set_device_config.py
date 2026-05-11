import firm_client


def test_set_device_config_response_mock() -> None:
    client, device = firm_client.FIRMClient.new_mock(timeout=0.01)
    client.start()
    try:
        # Inject a SetDeviceConfig response (identifier 0x0003, payload [1])
        device.inject_response(0x0003, bytes([1]))

        ok = client.set_device_config(
            "TestDevice",
            100,
            firm_client.DeviceProtocol.UART,
            timeout_seconds=0.1,
        )

        assert ok is True
    finally:
        client.stop()
