"""
Unit tests for FusedDataNode.

These tests test the filter logic directly without needing to
run a ROS2 system, real sensors, or any other simulation.

We initialise rclpy once for the whole module, create the node,
and call the callbacks manually with fake messages.
"""

import pytest
import rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from sensor_fusion_pkg.fused_data_node import FusedDataNode, GRAVITY



#helpers                                                                


def make_imu(az):
    """Return a minimal imu message with linear_acceleration.z set to az."""
    msg = Imu()
    msg.linear_acceleration.z = az
    return msg


def make_depth(value):
    """Return a Float32 depth message."""
    msg = Float32()
    msg.data = value
    return msg



#fixtures


@pytest.fixture(scope="module")
def ros():
    """Initialise and shut down rclpy once for the entire test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    """Create a fresh FusedDataNode for each test, destroy it after."""
    n = FusedDataNode()
    yield n
    n.destroy_node()



#tests                                                                    


def test_node_initialises(node):
    """Node should start with zeroed filter state."""
    assert node.v_fused == 0.0
    assert node.prev_depth is None
    assert node.prev_time is None
    assert node.latest_imu is None


def test_imu_callback_stores_message(node):
    """imu_callback should store the message without triggering fusion."""
    msg = make_imu(GRAVITY)
    node.imu_callback(msg)
    assert node.latest_imu is not None
    assert node.latest_imu.linear_acceleration.z == GRAVITY


def test_depth_callback_ignored_without_imu(node):
    """depth_callback should do nothing if no IMU message has arrived yet."""
    node.depth_callback(make_depth(10.0))
    #state should remain uninitialised
    assert node.prev_depth is None


def test_first_depth_message_initialises_state(node):
    """first depth message after IMU arrives should set prev_depth and prev_time."""
    node.imu_callback(make_imu(GRAVITY))
    node.depth_callback(make_depth(5.0))
    assert node.prev_depth == 5.0
    assert node.prev_time is not None


def test_stationary_velocity_is_near_zero(node):
    """
    with no real motion, v_fused should stay close to zero.

    when the IMU reads exactly GRAVITY (stationary), a_z_compensated = 0.
    v_imu = v_prev + 0 * dt = 0.
    when depth doesn't change, v_depth = 0.
    so v_fused should stay at 0.
    """
    node.imu_callback(make_imu(GRAVITY))  # a_z = 9.81 ; compensated = 0

    #first depth message just initialises state
    node.depth_callback(make_depth(10.0))

    #manually advance time so dt is valid
    node.prev_time -= 0.1  # simulate 100ms passing

    #second depth message at same depth ; v_depth = 0
    node.depth_callback(make_depth(10.0))

    assert abs(node.v_fused) < 1e-6


def test_descending_gives_positive_velocity(node):
    """increasing depth should produce a positive velocity estimate."""
    node.imu_callback(make_imu(GRAVITY))
    node.depth_callback(make_depth(10.0))

    node.prev_time -= 0.5  #simulate 1 second passing

    node.depth_callback(make_depth(11.0))  #descended 1 meter

    assert node.v_fused > 0.0


def test_ascending_gives_negative_velocity(node):
    """decreasing depth should produce a negative velocity estimate."""
    node.imu_callback(make_imu(GRAVITY))
    node.depth_callback(make_depth(10.0))

    node.prev_time -= 0.5

    node.depth_callback(make_depth(9.0))  #ascended 1 meter

    assert node.v_fused < 0.0


def test_stale_dt_resets_filter(node):
    """a dt larger than max_dt should reset v_fused to zero."""
    node.imu_callback(make_imu(GRAVITY))
    node.depth_callback(make_depth(10.0))

    #force a gap larger than max_dt to check stale values (default 1.0s)
    node.prev_time -= 5.0
    node.v_fused = 3.0  #pretend filter had built up a value

    node.depth_callback(make_depth(10.0))

    assert node.v_fused == 0.0


def test_alpha_parameter_default(node):
    """alpha should default to 0.98."""
    assert node.alpha == 0.98


def test_max_dt_parameter_default(node):
    """max_dt should default to 1.0."""
    assert node.max_dt == 1.0
