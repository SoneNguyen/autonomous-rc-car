"""Run record semantics: obstacle contact, ground exclusion, and trial status."""

from car_autonomy_nodes.run_logger import (
    REST_SETTLE_NS,
    allowlisted_obstacle_contact,
    collision_status,
    completion_status,
    is_ground_contact,
    new_run_id,
    RunLogger,
)
from rosgraph_msgs.msg import Clock


def test_ground_contact_never_counts_as_an_obstacle_collision():
    pair = ("gazebo_truck::chassis_collision", "ground::link::ground_collision")
    assert is_ground_contact(pair)
    assert not allowlisted_obstacle_contact(pair, "post_01_collision")
    assert collision_status(observed=False, coverage="verified") == "no_collision"


def test_only_the_named_non_ground_obstacle_is_allowlisted():
    target = ("gazebo_truck::chassis_collision", "course_obstacles::post_01_collision")
    other = ("gazebo_truck::chassis_collision", "course_obstacles::post_02_collision")
    assert allowlisted_obstacle_contact(target, "post_01_collision")
    assert not allowlisted_obstacle_contact(other, "post_01_collision")
    assert collision_status(observed=True, coverage="verified") == "collision"


def test_unknown_coverage_cannot_be_reported_as_no_collision():
    assert collision_status(observed=False, coverage="limited") == "unknown"
    assert collision_status(observed=False, coverage="stale") == "unknown"


def test_completion_timeout_and_abort_vocabulary_is_consistent():
    assert completion_status("rest_and_flush") == ("completed", "rest_and_flush")
    assert completion_status("simulation_time_limit") == (
        "timed_out", "simulation_time_limit"
    )
    for reason in ("simulation_reset", "shutdown", "competing_writer", "lighting_mismatch"):
        assert completion_status(reason) == ("aborted", reason)


def _clock_message(stamp_ns):
    message = Clock()
    message.clock.sec, message.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
    return message


def _clock_logger(*, timeout_s=60.0, stop_requested=False):
    logger = RunLogger.__new__(RunLogger)
    logger.start_sim_ns = None
    logger.last_clock_ns = None
    logger.scenario = {"run_timeout_sim_s": timeout_s}
    logger.speed_mps = 0.0
    logger.stop_request_stamp_ns = 1 if stop_requested else None
    logger.rest_since_ns = None
    logger.finished = False
    logger.finish_reasons = []
    logger._finish = logger.finish_reasons.append
    return logger


def test_sim_clock_completes_only_after_a_full_rest_settle_interval():
    logger = _clock_logger(stop_requested=True)
    logger._on_clock(_clock_message(10_000_000_000))
    assert logger.finish_reasons == []
    logger._on_clock(_clock_message(10_000_000_000 + REST_SETTLE_NS - 1))
    assert logger.finish_reasons == []
    logger._on_clock(_clock_message(10_000_000_000 + REST_SETTLE_NS))
    assert logger.finish_reasons == ["rest_and_flush"]
    assert completion_status(logger.finish_reasons[0])[0] == "completed"


def test_simulation_limit_and_backward_reset_end_as_distinct_statuses():
    timed_out = _clock_logger(timeout_s=2.0)
    timed_out._on_clock(_clock_message(5_000_000_000))
    timed_out._on_clock(_clock_message(7_000_000_000))
    assert completion_status(timed_out.finish_reasons[0]) == (
        "timed_out", "simulation_time_limit"
    )

    reset = _clock_logger(timeout_s=60.0)
    reset._on_clock(_clock_message(5_000_000_000))
    reset._on_clock(_clock_message(1_000_000_000))
    assert completion_status(reset.finish_reasons[0]) == ("aborted", "simulation_reset")


def test_each_retry_gets_a_fresh_run_id():
    assert new_run_id() != new_run_id()
