from kaiev26_aeb.trial_runtime import (
    PIPELINE_ISSUE_TIMEOUT_S, TARGET_LOSS_ISSUE_PREFIX, TARGET_LOSS_TIMEOUT_S, pipeline_issue_timeout,
)

TARGET_LOSS = TARGET_LOSS_ISSUE_PREFIX + ' short_path_or_no_forward_target'


def test_target_loss_alone_waits_longer_than_other_issues():
    assert pipeline_issue_timeout([TARGET_LOSS]) == TARGET_LOSS_TIMEOUT_S > PIPELINE_ISSUE_TIMEOUT_S
    assert pipeline_issue_timeout(['other issue']) == PIPELINE_ISSUE_TIMEOUT_S
    assert pipeline_issue_timeout([TARGET_LOSS, 'other issue']) == PIPELINE_ISSUE_TIMEOUT_S


def test_lost_mcap_subscription_never_stops_a_running_trial():
    from types import SimpleNamespace
    from kaiev26_aeb.trial_runtime import TrialNode

    node = TrialNode.__new__(TrialNode)
    node.settings = None
    node.received = {}
    node.json_status = {'/aeb/status': {'valid': True}}
    node.graph_scan = {'recorder_connected': False}
    node.enable_client = SimpleNamespace(service_is_ready=lambda: True)
    node.recorder_warned = False
    logs = []
    node.log = logs.append
    node.external_required = lambda settings: ()
    node._topic_issue = lambda *args, **kwargs: ''
    node._vehicle_issue = lambda now: ''
    node.command_flow_issue = lambda active: ''
    assert 'MCAP 필수 토픽 구독 대기' in node.pipeline_issues(0.0)
    assert node.pipeline_issues(0.0, recorder=False) == []
    assert node.pipeline_issues(0.0, recorder=False) == []
    assert len(logs) == 1 and '주행은 계속' in logs[0]
