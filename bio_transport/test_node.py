# test_node v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - 간단 기능 호출 테스트용 노드(필요 시 확장)

import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("bio_test")
    node.get_logger().info("bio_transport v1.000 test_node running")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
