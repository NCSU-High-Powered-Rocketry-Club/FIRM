import threading

import firm_client


def test_set_device_config_response_mock() -> None:
    client, device = firm_client.FIRMClient.new_mock(timeout=0.01)
    client.start()

    # Reply only after the client has sent its command, so the reply can't arrive first and be
    # discarded as stale.
    def respond() -> None:
        assert device.wait_for_command_identifier(timeout_seconds=1.0) == 0x04
        # SetDeviceConfig response (identifier 0x04, payload [1])
        device.inject_response(0x04, bytes([1]))

    responder = threading.Thread(target=respond)
    responder.start()
    try:
        ok = client.set_device_config(
            "TestDevice",
            100,
            firm_client.DeviceProtocol.UART,
            timeout_seconds=1.0,
        )

        assert ok is True
    finally:
        responder.join()
        client.stop()
