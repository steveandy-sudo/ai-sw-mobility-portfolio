"""Gate real-vehicle route tracking until its required inputs are healthy."""

from __future__ import annotations

from dataclasses import dataclass
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    RouteContext,
    SteeringActuatorStatus,
    VehicleState,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool


@dataclass
class InputState:
    """Latest health result and local receipt time for one required input."""

    received_ns: int = 0
    valid: bool = False
    detail: str = '메시지 미수신'


def finite_planar_covariance(
    covariance,
    y_variance_index: int,
    max_variance: float,
) -> bool:
    x_variance = float(covariance[0])
    y_variance = float(covariance[y_variance_index])
    return (
        math.isfinite(x_variance)
        and math.isfinite(y_variance)
        and 0.0 < x_variance <= max_variance
        and 0.0 < y_variance <= max_variance
    )


def covariance_detail(covariance, y_index: int, maximum: float, unit: str) -> str:
    return (
        f'분산 X={covariance[0]:.3g}, Y={covariance[y_index]:.3g} {unit}'
        f' (허용: 각 축 0 < 분산 <= {maximum:g} {unit})'
    )


class RouteReadinessNode(Node):
    """Report prerequisites and hold route commands until preflight passes."""

    INPUT_NAMES = (
        '/gnss/fix',
        '/gnss/fix_velocity',
        '/localization/odometry',
        '/vehicle/state',
        '/drive/status',
        '/steering/status',
        '/brake/status',
        '/planning/route_context',
    )
    UNIQUE_PUBLISHER_TOPICS = (
        '/gnss/fix',
        '/gnss/fix_velocity',
        '/localization/odometry',
        '/vehicle/state',
        '/drive/status',
        '/steering/status',
        '/brake/status',
        '/planning/route_context',
        '/planning/target_path',
        '/planning/target_speed',
        '/planning/command',
        '/vehicle/command',
    )

    def __init__(self) -> None:
        super().__init__('route_readiness')
        self.declare_parameter('freshness_timeout_sec', 1.0)
        self.declare_parameter('stable_ready_sec', 1.0)
        self.declare_parameter('maximum_position_variance_m2', 0.25)
        self.declare_parameter('maximum_velocity_variance_m2ps2', 1.0)
        self.declare_parameter('heading_initialization_speed_mps', 0.5)
        self.declare_parameter('maximum_start_speed_mps', 0.2)
        self.declare_parameter('maximum_cross_track_error_m', 2.0)
        self.declare_parameter(
            'maximum_heading_error_rad', math.radians(30.0)
        )
        self.declare_parameter('maximum_start_progress_m', 35.0)
        self.declare_parameter('allow_midroute_start', False)

        self.freshness_timeout_ns = int(
            1.0e9 * float(self.get_parameter('freshness_timeout_sec').value)
        )
        self.stable_ready_ns = int(
            1.0e9 * float(self.get_parameter('stable_ready_sec').value)
        )
        self.maximum_position_variance_m2 = float(
            self.get_parameter('maximum_position_variance_m2').value
        )
        self.maximum_velocity_variance_m2ps2 = float(
            self.get_parameter('maximum_velocity_variance_m2ps2').value
        )
        self.heading_initialization_speed_mps = float(
            self.get_parameter('heading_initialization_speed_mps').value
        )
        self.maximum_start_speed_mps = float(
            self.get_parameter('maximum_start_speed_mps').value
        )
        self.maximum_cross_track_error_m = float(
            self.get_parameter('maximum_cross_track_error_m').value
        )
        self.maximum_heading_error_rad = float(
            self.get_parameter('maximum_heading_error_rad').value
        )
        self.maximum_start_progress_m = float(
            self.get_parameter('maximum_start_progress_m').value
        )
        self.allow_midroute_start = bool(
            self.get_parameter('allow_midroute_start').value
        )

        self.inputs = {name: InputState() for name in self.INPUT_NAMES}
        self.reported_states: dict[str, bool] = {}
        self.latest_vehicle: VehicleState | None = None
        self.preflight_since_ns: int | None = None
        self.preflight_passed = False
        self.last_ready = False
        self.auto_reported = False
        self.heading_observed = False

        self.ready_pub = self.create_publisher(Bool, '/planning/route_ready', 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, '/planning/readiness_status', 10
        )
        self.create_subscription(
            NavSatFix, '/gnss/fix', self.on_fix, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/gnss/fix_velocity',
            self.on_velocity,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleState,
            '/vehicle/state',
            self.on_vehicle,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DriveActuatorStatus,
            '/drive/status',
            self.on_drive,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SteeringActuatorStatus,
            '/steering/status',
            self.on_steering,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BrakeActuatorStatus,
            '/brake/status',
            self.on_brake,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RouteContext,
            '/planning/route_context',
            self.on_route,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.evaluate)

    def update_input(self, name: str, valid: bool, detail: str) -> None:
        self.inputs[name] = InputState(
            received_ns=self.get_clock().now().nanoseconds,
            valid=valid,
            detail=detail,
        )

    def on_fix(self, message: NavSatFix) -> None:
        fixed = message.status.status == NavSatStatus.STATUS_GBAS_FIX
        covariance_good = (
            message.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
            and finite_planar_covariance(
                message.position_covariance,
                4,
                self.maximum_position_variance_m2,
            )
        )
        reasons = []
        if not fixed:
            reasons.append(
                f'RTK FIX 미확보: status={message.status.status}, 필요=2; '
                'GNSS 수신기와 RTK 보정 수신 상태 확인'
            )
        if message.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            reasons.append('위치 정확도 정보 없음')
        elif not covariance_good:
            reasons.append('위치 정확도 조건 미달: ' + covariance_detail(
                message.position_covariance, 4, self.maximum_position_variance_m2, 'm²'
            ))
        self.update_input(
            '/gnss/fix', fixed and covariance_good,
            '; '.join(reasons) if reasons else 'RTK FIX 확보, 위치 정확도 통과',
        )

    def on_velocity(self, message: TwistWithCovarianceStamped) -> None:
        velocity = message.twist.twist.linear
        covariance = message.twist.covariance
        reasons = []
        if not (math.isfinite(velocity.x) and math.isfinite(velocity.y)):
            reasons.append(f'GNSS 속도값 오류: ENU=({velocity.x}, {velocity.y}) m/s')
        if not finite_planar_covariance(
            covariance, 7, self.maximum_velocity_variance_m2ps2
        ):
            reasons.append('GNSS 속도 정확도 조건 미달: ' + covariance_detail(
                covariance, 7, self.maximum_velocity_variance_m2ps2, 'm²/s²'
            ))
        measurement_valid = not reasons
        speed_mps = math.hypot(velocity.x, velocity.y)
        if measurement_valid and speed_mps >= self.heading_initialization_speed_mps:
            self.heading_observed = True
        if measurement_valid and not self.heading_observed:
            fix = self.inputs['/gnss/fix']
            fix_ready = (
                fix.valid and fix.received_ns > 0
                and self.get_clock().now().nanoseconds - fix.received_ns
                <= self.freshness_timeout_ns
            )
            reasons.append(
                f'헤딩 초기화 대기: 현재 {speed_mps:.2f} m/s; '
                f'GNSS 조건 충족 후 MANUAL 전진 >= '
                f'{self.heading_initialization_speed_mps:.2f} m/s 필요, 이후 정지'
                if fix_ready else '헤딩 초기화 대기: GNSS 출발 조건부터 해결 필요'
            )
        self.update_input(
            '/gnss/fix_velocity',
            measurement_valid and self.heading_observed,
            '; '.join(reasons) if reasons else f'헤딩 초기화 통과, GNSS 속도 {speed_mps:.2f} m/s',
        )

    def on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        reasons = []
        if message.header.frame_id != 'map' or message.child_frame_id != 'base_footprint':
            reasons.append(
                f'좌표계 불일치: {message.header.frame_id} -> {message.child_frame_id}; '
                '필요: map -> base_footprint'
            )
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            reasons.append(f'위치값 오류: X={position.x}, Y={position.y}')
        if not finite_planar_covariance(
            message.pose.covariance, 7, self.maximum_position_variance_m2
        ):
            reasons.append('위치 정확도 조건 미달: ' + covariance_detail(
                message.pose.covariance, 7, self.maximum_position_variance_m2, 'm²'
            ))
        self.update_input(
            '/localization/odometry',
            not reasons,
            '; '.join(reasons) if reasons else 'map -> base_footprint, 위치 정확도 통과',
        )

    def on_vehicle(self, message: VehicleState) -> None:
        self.latest_vehicle = message
        reasons = []
        if not math.isfinite(message.speed_mps):
            reasons.append(f'차량 속도값 오류: {message.speed_mps}')
        if message.estop_active:
            reasons.append('물리 E-Stop 활성: 해제 필요')
        if not self.preflight_passed:
            if message.mode != VehicleState.MODE_MANUAL:
                reasons.append('출발 조건 확인 전 AUTO 전환됨: MANUAL로 복귀 필요')
            if abs(float(message.speed_mps)) > self.maximum_start_speed_mps:
                reasons.append(
                    f'차량 정지 필요: 현재 {message.speed_mps:.2f} m/s, '
                    f'허용 |속도| <= {self.maximum_start_speed_mps:.2f} m/s'
                )
        mode = 'AUTO' if message.mode == VehicleState.MODE_AUTO else 'MANUAL'
        self.update_input('/vehicle/state', not reasons, '; '.join(reasons) or f'{mode}, 차량 상태 통과')

    def on_drive(self, message: DriveActuatorStatus) -> None:
        reasons = []
        if message.fault_bits_valid and message.fault_bits != 0:
            reasons.append(f'구동기 고장 보고: fault_bits=0x{message.fault_bits:x}')
        if not self.preflight_passed and message.mode != message.MODE_MANUAL:
            reasons.append('출발 조건 확인 전 구동기가 AUTO 상태: MANUAL로 복귀 필요')
        self.update_input(
            '/drive/status',
            not reasons,
            '; '.join(reasons) or '구동기 상태 통과',
        )

    def on_steering(self, message: SteeringActuatorStatus) -> None:
        reasons = []
        warnings = []
        if not message.communication_ok:
            reasons.append('조향 CAN 통신 불량: 전원·배선 확인')
        if not message.feedback_valid:
            reasons.append('조향각 피드백 무효: 센서·피드백 연결 확인')
        if message.can_timeout:
            reasons.append('조향 CAN 수신 시간 초과')
        if message.estop_active:
            reasons.append('조향 E-Stop 활성')
        if message.tracking_error_fault:
            warnings.append('조향 추종 오차 경고: 시험과 기록은 계속 진행')
        details = reasons + warnings
        self.update_input(
            '/steering/status',
            not reasons,
            '; '.join(details) or '조향 CAN·피드백 통과',
        )

    def on_brake(self, _message: BrakeActuatorStatus) -> None:
        self.update_input('/brake/status', True, '브레이크 상태 수신')

    def on_route(self, message: RouteContext) -> None:
        reasons = []
        if not message.route_projection_valid:
            reasons.append('차량 위치의 경로 투영 실패: 선택 경로와 차량 위치·방향 확인')
        if not abs(float(message.cross_track_error)) <= self.maximum_cross_track_error_m:
            reasons.append(
                f'횡오차 {message.cross_track_error:+.2f} m; '
                f'허용 |횡오차| <= {self.maximum_cross_track_error_m:.2f} m'
            )
        if not abs(float(message.heading_error)) <= self.maximum_heading_error_rad:
            reasons.append(
                f'헤딩 오차 {math.degrees(message.heading_error):+.1f}°; '
                f'허용 |헤딩 오차| <= {math.degrees(self.maximum_heading_error_rad):.1f}°'
            )
        if len(message.local_path_points) < 2:
            reasons.append(f'추종 경로점 부족: {len(message.local_path_points)}개, 최소 2개 필요')
        if (
            not self.preflight_passed and not self.allow_midroute_start
            and not message.progress_s <= self.maximum_start_progress_m
        ):
            reasons.append(
                f'출발 위치 범위 초과: 진행거리 {message.progress_s:.1f} m, '
                f'허용 <= {self.maximum_start_progress_m:.1f} m'
            )
        self.update_input(
            '/planning/route_context',
            not reasons,
            '; '.join(reasons) if reasons else (
                f'경로 정합 통과: 횡오차 {message.cross_track_error:+.2f} m, '
                f'헤딩 오차 {math.degrees(message.heading_error):+.1f}°'
            ),
        )

    def current_results(self, now_ns: int) -> dict[str, tuple[bool, str]]:
        results = {}
        for name, state in self.inputs.items():
            fresh = (
                state.received_ns > 0
                and now_ns - state.received_ns <= self.freshness_timeout_ns
            )
            results[name] = (
                state.valid and fresh,
                state.detail if fresh else (
                    f'수신 중단: 마지막 수신 {(now_ns - state.received_ns) / 1e9:.1f}초 전, '
                    f'허용 {self.freshness_timeout_ns / 1e9:g}초'
                    if state.received_ns > 0 else '메시지 미수신: 해당 노드와 토픽 연결 확인'
                ),
            )
        for topic in self.UNIQUE_PUBLISHER_TOPICS:
            publisher_count = self.count_publishers(topic)
            count_valid = publisher_count == 1
            count_detail = f'publishers={publisher_count}; 발행자 정확히 1개 필요'
            if topic in results:
                input_valid, input_detail = results[topic]
                results[topic] = (
                    input_valid and count_valid,
                    input_detail if count_valid else f'{input_detail}; {count_detail}',
                )
            else:
                results[topic] = (count_valid, '발행자 1개 확인' if count_valid else count_detail)
        return results

    def report(self, name: str, valid: bool, detail: str) -> None:
        if self.reported_states.get(name) == valid:
            return
        self.reported_states[name] = valid
        label = 'GOOD' if valid else 'WAIT'
        if valid:
            self.get_logger().info(f'{name} : {label} - {detail}')
        else:
            self.get_logger().warning(f'{name} : {label} - {detail}')

    def publish_status(
        self,
        results: dict[str, tuple[bool, str]],
        runtime_ready: bool,
    ) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()

        aggregate = DiagnosticStatus()
        aggregate.name = 'PRE-FLIGHT'
        aggregate.hardware_id = 'kaiev26'
        aggregate.level = (
            DiagnosticStatus.OK
            if runtime_ready
            else DiagnosticStatus.ERROR
            if self.preflight_passed
            else DiagnosticStatus.WARN
        )
        aggregate.message = (
            'GOOD'
            if runtime_ready
            else 'FAULT'
            if self.preflight_passed
            else 'WAIT'
        )
        message.status.append(aggregate)

        for name, (valid, detail) in results.items():
            status = DiagnosticStatus()
            status.name = name
            status.hardware_id = 'kaiev26'
            status.level = (
                DiagnosticStatus.OK
                if valid
                else DiagnosticStatus.ERROR
                if self.preflight_passed
                else DiagnosticStatus.WARN
            )
            status.message = (
                'GOOD'
                if valid
                else 'FAULT'
                if self.preflight_passed
                else 'WAIT'
            )
            status.values = [KeyValue(key='detail', value=detail)]
            message.status.append(status)
        self.status_pub.publish(message)

    def evaluate(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        results = self.current_results(now_ns)
        for name, (valid, detail) in results.items():
            self.report(name, valid, detail)

        all_good = all(valid for valid, _detail in results.values())
        if not self.preflight_passed:
            if all_good:
                if self.preflight_since_ns is None:
                    self.preflight_since_ns = now_ns
                elif now_ns - self.preflight_since_ns >= self.stable_ready_ns:
                    self.preflight_passed = True
                    self.get_logger().info(
                        'PRE-FLIGHT : READY - safety driver may now switch to AUTO'
                    )
            else:
                self.preflight_since_ns = None

        runtime_ready = self.preflight_passed and all_good
        self.ready_pub.publish(Bool(data=runtime_ready))
        self.publish_status(results, runtime_ready)
        if self.last_ready and not runtime_ready:
            self.get_logger().error(
                'ROUTE TRACKING : BLOCKED - a required input became unhealthy'
            )
        self.last_ready = runtime_ready

        if (
            runtime_ready
            and self.latest_vehicle is not None
            and self.latest_vehicle.mode == VehicleState.MODE_AUTO
            and not self.auto_reported
        ):
            self.auto_reported = True
            self.get_logger().info('AUTO : ACTIVE - constant-speed tracking enabled')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RouteReadinessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
