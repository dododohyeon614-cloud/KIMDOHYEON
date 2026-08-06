#!/usr/bin/env python3
"""
prompt.py - 사용자 명령 인터페이스 노드
기능: 매핑, 주행, 저장된 맵 목록 확인, 프로그램 종료 기능 제공
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import os
import sys

class PromptNode(Node):
    def __init__(self):
        super().__init__('prompt_node')
        self.map_pub = self.create_publisher(Bool, '/mapping_cmd', 10)
        self.drive_pub = self.create_publisher(String, '/drive_cmd', 10)
        self.state_pub = self.create_publisher(Bool, '/control_state', 10)
        self.data_dir = os.path.expanduser("~/white_ws/gps_data")
        # 🔧 BUG7 수정: 워커스레드에서 메인루프 종료 신호를 전달하는 이벤트
        self._shutdown_event = threading.Event()

    def get_input(self):
        print("\n=============================")
        print(" 1. 매핑 시작 (Enter로 종료/저장)")
        print(" 2. 경로 주행 시작 (Enter로 정지)")
        print(" 3. 저장된 경로 목록 확인")
        print(" 4. 터미널 종료 (Exit)")
        print("=============================")
        return input("메뉴 선택 (1/2/3/4): ").strip()

    def list_routes(self):
        if not os.path.exists(self.data_dir):
            print(f"⚠️ {self.data_dir} 폴더가 없습니다.")
            return []
        
        files = sorted([f for f in os.listdir(self.data_dir) if f.endswith('.csv')])
        if not files:
            print("⚠️ 저장된 경로(.csv) 파일이 없습니다.")
            return []
        
        print("\n📂 [저장된 경로 파일 목록]")
        for i, f in enumerate(files):
            print(f" {i+1}. {f}")
        return files

    def run(self):
        while rclpy.ok():
            choice = self.get_input()

            if choice == '1':
                print("\n🗺️ 매핑을 시작합니다...")
                self.map_pub.publish(Bool(data=True))
                input("🛑 매핑을 종료하고 저장하려면 [Enter] 키를 누르세요...\n")
                self.map_pub.publish(Bool(data=False))
                print("✅ 매핑 종료 및 저장 완료!")

            elif choice == '2':
                files = self.list_routes()
                if not files:
                    continue
                
                try:
                    f_idx = int(input("\n주행할 파일 번호: ")) - 1
                    if 0 <= f_idx < len(files):
                        target_file = files[f_idx]
                        
                        # 🌟 [수정 포인트 2] 주행 시작 시 모터 제어 권한(True) 부여
                        self.state_pub.publish(Bool(data=True))
                        self.drive_pub.publish(String(data=target_file))
                        print(f"\n🚀 자율주행 시작: {target_file}")
                        
                        input("🛑 주행을 긴급 정지하려면 [Enter] 키를 누르세요...\n")
                        
                        # 🌟 [수정 포인트 3] 긴급 정지 시 모터 제어 권한(False) 즉각 박탈
                        self.state_pub.publish(Bool(data=False))
                        self.drive_pub.publish(String(data="STOP"))
                        print("🛑 긴급 정지 명령 및 모터 차단 완료!")
                    else:
                        print("⚠️ 잘못된 번호입니다.")
                except ValueError:
                    print("⚠️ 숫자를 입력하세요.")

            elif choice == '3':
                self.list_routes()

            elif choice == '4':
                print("\n👋 터미널을 종료합니다.")
                # 🔧 BUG7 수정: 워커스레드에서 직접 rclpy.shutdown()+sys.exit() 호출하면
                # spin() 중인 메인스레드와 충돌 → 이벤트로 신호만 보내고 메인이 처리
                self._shutdown_event.set()
                return

            else:
                print("⚠️ 1번에서 4번 사이의 숫자를 입력해주세요.")

def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()
    
    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()

    try:
        # 🔧 BUG7 수정: spin 대신 주기적으로 shutdown_event를 확인하며 spin_once
        while rclpy.ok() and not node._shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.state_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()