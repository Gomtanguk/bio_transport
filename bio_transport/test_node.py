# test_node v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 테스트 노드 목적/확장 포인트 주석 추가(기능 변경 없음)

# [모듈 역할]
# - 개발/통신 확인용 최소 노드
# - 필요 시 여기서 토픽/서비스/액션 테스트를 빠르게 추가
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("bio_test")
    node.get_logger().info("bio_transport v2.000 test_node running")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
