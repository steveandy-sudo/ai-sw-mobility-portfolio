"""The four K-City cone-course perception and tracking combinations."""

from dataclasses import asdict, dataclass
import math
import os
from pathlib import Path


COUNTDOWN_SEC = 5.0


def default_model_path() -> str:
    return os.environ.get(
        'KAIEV26_CONE_MODEL',
        str(Path.home() / 'KAI_ws/models/cone_yolo/yolov8n-cone.pt'))


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
        1, 'fusion', 'stanley', '라바콘 퓨전', 'Stanley',
        'cone_path_node', 'cone_stanley_node'),
    2: TrialProfile(
        2, 'fusion', 'pure_pursuit', '라바콘 퓨전', 'Pure Pursuit',
        'cone_path_node', 'cone_pursuit_node'),
    3: TrialProfile(
        3, 'yolo', 'stanley', '라바콘 YOLO', 'Stanley',
        'cone_yolo_path_node', 'cone_stanley_node'),
    4: TrialProfile(
        4, 'yolo', 'pure_pursuit', '라바콘 YOLO', 'Pure Pursuit',
        'cone_yolo_path_node', 'cone_pursuit_node'),
}


def profile_for(scenario: int) -> TrialProfile:
    try:
        return PROFILES[int(scenario)]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('시나리오는 1..4입니다') from error


@dataclass(frozen=True)
class TrialSettings:
    scenario: int = 2
    target_speed_kph: float = 5.0
    model_path: str = ''

    def __post_init__(self):
        profile_for(self.scenario)
        if not math.isfinite(self.target_speed_kph) or self.target_speed_kph <= 0:
            raise ValueError('최고 목표속도는 0보다 큰 유한한 값이어야 합니다')
        if not self.model_path:
            raise ValueError('YOLO 모델 경로가 필요합니다')

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
        })
        return result
