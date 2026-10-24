# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for spine halt, re-arm, mid-move cancellation and read throttling."""

import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from franka_spine_server.spine_api import SpineMotion, SpineStatus
from franka_spine_server.spine_api_client import SpineApiClient
from franka_spine_server.spine_controller import DEFAULT_RETRY_DELAY, SpineController


def _make_controller(feedback_rate=100.0, retry_delay=DEFAULT_RETRY_DELAY):
    """Build a controller with a mocked SpineApiClient."""
    api = MagicMock()
    api.timeout = 1.0
    with patch(
        'franka_spine_server.spine_controller.SpineApiClient',
        return_value=api,
    ):
        controller = SpineController(
            '172.16.16.10',
            timeout=1.0,
            feedback_rate=feedback_rate,
            retry_delay=retry_delay,
        )
    return controller, api


class TestHaltMotion(unittest.TestCase):
    """Halt path, Halt.srv re-arm, and move_absolute cancellation."""

    def test_halt_motion_posts_quick_stop(self):
        client = SpineApiClient('172.16.16.10', timeout=1.0)
        client.post = MagicMock(return_value=(True, SpineStatus.QuickStopActive))

        success, data = client.halt_motion()

        self.assertTrue(success)
        self.assertEqual(data, SpineStatus.QuickStopActive)
        client.post.assert_called_once_with('motion:quick-stop')

    def test_halt_rearms_to_switched_on(self):
        controller, api = _make_controller()
        api.halt_motion.return_value = (True, SpineStatus.QuickStopActive)
        api.get_state.return_value = (True, SpineStatus.SwitchedOff)
        api.switch_on.return_value = (True, SpineStatus.SwitchedOn)

        result = controller.halt()

        self.assertTrue(result.success)
        self.assertEqual(result.state, SpineStatus.SwitchedOn)
        self.assertEqual(result.message, 'Halt successful')
        api.halt_motion.assert_called_once()
        api.switch_on.assert_called_once()

    def test_halt_does_not_rearm_when_quick_stop_fails(self):
        controller, api = _make_controller()
        api.halt_motion.return_value = (
            False,
            '404 Client Error: Not Found for url: '
            'https://172.16.16.10/spine/api/motion:halt',
        )

        result = controller.halt()

        self.assertFalse(result.success)
        self.assertIn('404', result.message)
        api.switch_on.assert_not_called()

    def test_move_absolute_publishes_feedback_while_start_blocks(self):
        controller, api = _make_controller()
        started = threading.Event()
        saw_feedback = threading.Event()
        positions = []

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(saw_feedback.wait(timeout=2.0))
            return True, SpineMotion.Finished

        api.start_motion.side_effect = blocking_start
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(position):
            positions.append(position)
            if started.is_set():
                saw_feedback.set()

        result = controller.move_absolute(0.45, 0.02, 0.2, 0.2, on_feedback=on_feedback)

        self.assertTrue(result.success)
        self.assertEqual(result.stop_by, SpineMotion.Finished)
        self.assertTrue(positions)
        self.assertTrue(started.is_set())

    def test_cancel_halts_while_start_motion_is_in_flight(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}
        positions = []

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, 'QuickStop'

        state = {'value': SpineStatus.SwitchedOn}

        def halt_motion():
            state['value'] = SpineStatus.SwitchedOff
            release.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.side_effect = lambda: (True, state['value'])
        api.switch_on.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(position):
            positions.append(position)
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.372,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertTrue(result.cancelled)
        self.assertFalse(result.success)
        self.assertEqual(result.error, 'Motion cancelled')
        self.assertTrue(positions)
        api.halt_motion.assert_called()
        api.switch_on.assert_called_once()

    def test_cancel_does_not_report_canceled_when_halt_fails(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, SpineMotion.Finished

        def halt_motion():
            release.set()
            return False, '404 Client Error: Not Found'

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.5})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.55,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertFalse(result.cancelled)
        self.assertFalse(result.success)
        self.assertIn('404', result.error)
        api.switch_on.assert_not_called()

    def test_cancel_after_start_reaches_target_skips_rearm_and_succeeds(self):
        controller, api = _make_controller()
        started = threading.Event()
        entered_halt = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(entered_halt.wait(timeout=2.0))
            return True, SpineMotion.Finished

        def halt_motion():
            entered_halt.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.55})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.55,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertTrue(result.success)
        self.assertFalse(result.cancelled)
        self.assertEqual(result.stop_by, SpineMotion.Finished)
        api.halt_motion.assert_called()
        api.switch_on.assert_not_called()

    def test_cancel_rearm_failure_does_not_report_canceled(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, 'QuickStop'

        state = {'value': SpineStatus.SwitchedOn}

        def halt_motion():
            state['value'] = SpineStatus.SwitchedOff
            release.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.side_effect = lambda: (True, state['value'])
        api.switch_on.return_value = (False, 'drive still off')

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.372,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertFalse(result.cancelled)
        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        api.switch_on.assert_called_once()

    def test_move_absolute_rejects_when_switched_off(self):
        controller, api = _make_controller()
        api.get_state.return_value = (True, SpineStatus.SwitchedOff)

        result = controller.move_absolute(0.2, 0.01, 0.2, 0.2)

        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOff, result.error)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        self.assertNotIn('424', result.error)
        api.start_motion.assert_not_called()
        api.halt_motion.assert_not_called()

    def test_move_absolute_maps_424_without_http_details(self):
        controller, api = _make_controller()
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)
        api.get_position.return_value = (True, {'position': 0.321})
        api.start_motion.return_value = (
            False,
            '424 Client Error: Failed Dependency for url: '
            'https://172.16.16.10/spine/api/motion-mm:start',
        )

        result = controller.move_absolute(0.2, 0.01, 0.2, 0.2)

        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        self.assertNotIn('424', result.error)
        api.halt_motion.assert_not_called()


class TestReadThrottling(unittest.TestCase):
    """Reads must never pile up on the device, however hard clients poll."""

    def test_concurrent_reads_issue_a_single_request(self):
        controller, api = _make_controller(feedback_rate=1.0)
        results = []
        append_lock = threading.Lock()

        def slow_read():
            time.sleep(0.05)
            return True, {'position': 0.42}

        api.get_position.side_effect = slow_read

        def poll():
            result = controller.get_position()
            with append_lock:
                results.append(result)

        threads = [threading.Thread(target=poll) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)

        self.assertEqual(api.get_position.call_count, 1)
        self.assertEqual(len(results), 8)
        self.assertTrue(all(result.success for result in results))
        self.assertTrue(all(result.position == 0.42 for result in results))

    def test_repeated_reads_within_one_period_do_not_reach_the_device(self):
        controller, api = _make_controller(feedback_rate=1.0)
        api.get_position.return_value = (True, {'position': 0.31})

        for _ in range(50):
            self.assertTrue(controller.get_position().success)

        self.assertEqual(api.get_position.call_count, 1)

    def test_sample_expires_after_one_feedback_period(self):
        controller, api = _make_controller(feedback_rate=1000.0)
        api.get_position.return_value = (True, {'position': 0.31})

        controller.get_position()
        time.sleep(0.01)
        controller.get_position()

        self.assertEqual(api.get_position.call_count, 2)

    def test_read_in_flight_serves_the_previous_sample(self):
        controller, api = _make_controller(feedback_rate=1000.0)
        in_flight = threading.Event()
        release = threading.Event()
        calls = []

        def read():
            calls.append(None)
            if len(calls) == 1:
                return True, {'position': 0.11}
            in_flight.set()
            release.wait(timeout=2.0)
            return True, {'position': 0.22}

        api.get_position.side_effect = read
        self.assertEqual(controller.get_position().position, 0.11)

        time.sleep(0.01)
        blocked = threading.Thread(target=controller.get_position)
        blocked.start()
        self.assertTrue(in_flight.wait(timeout=2.0))

        self.assertEqual(controller.get_position().position, 0.11)
        self.assertEqual(api.get_position.call_count, 2)

        release.set()
        blocked.join(timeout=2.0)

    def test_failed_read_is_not_retried_before_the_retry_delay(self):
        controller, api = _make_controller(feedback_rate=1000.0, retry_delay=0.2)
        api.get_position.return_value = (False, 'read timeout=2.0')

        for _ in range(20):
            self.assertFalse(controller.get_position().success)
        self.assertEqual(api.get_position.call_count, 1)

        time.sleep(0.25)
        controller.get_position()

        self.assertEqual(api.get_position.call_count, 2)

    def test_reads_return_to_full_rate_after_recovery(self):
        controller, api = _make_controller(feedback_rate=1000.0, retry_delay=0.05)
        api.get_position.side_effect = [
            (False, 'read timeout=2.0'),
            (True, {'position': 0.4}),
            (True, {'position': 0.4}),
        ]

        self.assertFalse(controller.get_position().success)
        time.sleep(0.06)
        self.assertTrue(controller.get_position().success)
        time.sleep(0.01)
        self.assertTrue(controller.get_position().success)

        self.assertEqual(api.get_position.call_count, 3)

    def test_failed_reads_are_logged_once_per_request(self):
        controller, api = _make_controller(feedback_rate=1.0, retry_delay=5.0)
        controller.logger = MagicMock()
        api.get_position.return_value = (False, 'read timeout=2.0')

        for _ in range(20):
            controller.get_position()

        self.assertEqual(api.get_position.call_count, 1)
        controller.logger.error.assert_called_once()
        self.assertIn('get_position failed', controller.logger.error.call_args[0][0])

    def test_command_during_an_in_flight_read_discards_its_result(self):
        controller, api = _make_controller(feedback_rate=1.0)
        in_flight = threading.Event()
        release = threading.Event()
        reads = [SpineStatus.SwitchedOn, SpineStatus.SwitchedOff]

        def blocking_state():
            if len(reads) == 2:
                in_flight.set()
                release.wait(timeout=2.0)
            return True, reads.pop(0)

        api.get_state.side_effect = blocking_state
        api.switch_off.return_value = (True, SpineStatus.SwitchedOff)

        served = []
        reader = threading.Thread(target=lambda: served.append(controller.get_state()))
        reader.start()
        self.assertTrue(in_flight.wait(timeout=2.0))

        controller.switch_off()
        release.set()
        reader.join(timeout=2.0)

        self.assertEqual(served[0].state, SpineStatus.SwitchedOn)
        self.assertEqual(controller.get_state().state, SpineStatus.SwitchedOff)

    def test_switch_on_invalidates_the_cached_state(self):
        controller, api = _make_controller(feedback_rate=1.0)
        api.get_state.return_value = (True, SpineStatus.SwitchedOff)
        self.assertEqual(controller.get_state().state, SpineStatus.SwitchedOff)

        api.switch_on.return_value = (True, SpineStatus.SwitchedOn)
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)
        controller.switch_on()

        self.assertEqual(controller.get_state().state, SpineStatus.SwitchedOn)


if __name__ == '__main__':
    unittest.main()
