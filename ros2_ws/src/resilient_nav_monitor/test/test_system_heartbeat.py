# Copyright 2026 Kylian
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

from resilient_nav_monitor.system_heartbeat import format_heartbeat
from resilient_nav_monitor.system_heartbeat import HEARTBEAT_PERIOD_SECONDS
from resilient_nav_monitor.system_heartbeat import HEARTBEAT_TOPIC


def test_heartbeat_contract():
    assert HEARTBEAT_TOPIC == '/system_heartbeat'
    assert HEARTBEAT_PERIOD_SECONDS == 1.0


def test_heartbeat_payload_contains_alive_and_incrementing_count():
    payloads = [format_heartbeat(count) for count in range(1, 4)]

    assert payloads == [
        'alive count=1',
        'alive count=2',
        'alive count=3',
    ]
