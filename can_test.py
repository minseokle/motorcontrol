import can

def receive_can_messages_on_com10():
    """
    Windows의 COM10에 연결된 CANable(slcan)을 통해 CAN 메시지를 수신합니다.
    """
    bus = None  # bus 변수를 미리 선언
    try:
        # 'slcan' 인터페이스를 사용하여 COM10 포트로 CAN 버스를 초기화합니다.
        # bitrate는 통신하려는 CAN 네트워크의 속도에 맞춰 설정해야 합니다 (예: 500000).
        bus = can.interface.Bus(bustype='slcan', channel='COM3', bitrate=1000000)
        
        print(f"COM10({bus.channel_info})에서 CAN 메시지 수신을 시작합니다...")
        print("프로그램을 종료하려면 Ctrl+C를 누르세요.")
        
        # bus 객체는 이터레이터(iterator)처럼 동작하여, 메시지가 수신될 때마다 반환합니다.
        for msg in bus:
            print(f"수신된 메시지: {msg}")

    except can.CanError as e:
        print(f"CAN 오류 발생: {e}")
        print("다음 사항을 확인하세요:")
        print("1. CANable 장치가 COM10에 올바르게 연결되었는지 확인하세요.")
        print("2. 장치 관리자에서 COM 포트 번호가 정확한지 확인하세요.")
        print("3. 다른 프로그램이 COM10 포트를 사용하고 있지 않은지 확인하세요.")
        print("4. Bitrate가 CAN 네트워크 설정과 일치하는지 확인하세요.")
        
    except KeyboardInterrupt:
        # Ctrl+C를 눌러 프로그램을 종료할 때 실행됩니다.
        print("\n프로그램을 종료합니다.")
        
    finally:
        # 버스 연결을 안전하게 종료합니다.
        if bus is not None:
            bus.shutdown()
            print("CAN 버스 연결이 종료되었습니다.")


if __name__ == "__main__":
    receive_can_messages_on_com10()