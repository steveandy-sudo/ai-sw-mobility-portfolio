"""K-City brake-inspection cases and run settings."""

from dataclasses import asdict, dataclass, field
import math
from pathlib import Path


COUNTDOWN_SEC = 5.0
TRIAL_ACCELERATION_MPS2 = 1.0
# The trial TUI runs the straight brake course with Stanley only; Pure Pursuit stays in the launch.
TRIAL_SCENARIO = 2


def installed_cone_engine_path() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('cone_perception')) / 'models/yolov8n-cone.engine'
        return path if path.is_file() else None
    except (ImportError, LookupError, OSError):
        return None


@dataclass(frozen=True)
class TrialProfile:
    scenario: int
    perception: str
    controller: str
    perception_label: str
    controller_label: str
    perception_node: str
    controller_node: str

    @property
    def name(self) -> str:
        return f'{self.perception_label} + {self.controller_label}'

    @property
    def required_launches(self) -> tuple[str]:
        return ('aeb_test.launch.py',)

    def missing_launches(self, launch_dir: Path) -> tuple[str, ...]:
        return tuple(name for name in self.required_launches
                     if not (Path(launch_dir) / name).is_file())


PROFILES = {
    1: TrialProfile(
        1, 'colored', 'pure_pursuit', '컬러 콘 3D 인지', 'Pure Pursuit',
        'cone_colored_path_node', 'cone_pursuit_node'),
    2: TrialProfile(
        2, 'colored', 'stanley', '컬러 콘 3D 인지', 'Stanley',
        'cone_colored_path_node', 'cone_stanley_node'),
}


def profile_for(scenario: int) -> TrialProfile:
    try:
        return PROFILES[int(scenario)]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('Case는 1 또는 2입니다') from error


def default_target_speed_kph(scenario: int) -> float:
    profile_for(scenario)
    return 30.0


@dataclass(frozen=True)
class TrialSettings:
    scenario: int = TRIAL_SCENARIO
    target_speed_kph: float = 30.0
    acceleration_mps2: float = field(default=TRIAL_ACCELERATION_MPS2, init=False)
    red_brake_lead_m: float = 5.0
    # USE: a person stops the car with the E-stop at the red line; the controller neither slows
    # nor brakes for it. NOT-USE (default): the controller stops and brakes at the red line.
    estop_use: bool = False

    def __post_init__(self):
        profile_for(self.scenario)
        if not math.isfinite(self.target_speed_kph) or self.target_speed_kph <= 0:
            raise ValueError('최고 목표속도는 0보다 큰 유한한 값이어야 합니다')
        if not math.isfinite(self.red_brake_lead_m) or self.red_brake_lead_m < 0:
            raise ValueError('선행 제동거리는 0 이상이어야 합니다')

    @property
    def profile(self) -> TrialProfile:
        return profile_for(self.scenario)

    def document(self) -> dict:
        result = asdict(self)
        result.update({
            'scenario_name': self.profile.name,
            'perception': self.profile.perception,
            'controller': self.profile.controller,
            'countdown_sec': COUNTDOWN_SEC,
            'command_topic': '/planning/command',
            'estop': 'USE' if self.estop_use else 'NOT-USE',
            'vehicle_brake': not self.estop_use,
        })
        return result
