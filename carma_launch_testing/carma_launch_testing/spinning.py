# Copyright 2023 Leidos
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

import time
from typing import Callable

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .exceptions import NodeSpinTimeoutError

Predicate = Callable[[], bool]


def spin_node_until(
    node: Node, predicate: Predicate, ros_context: Context, timeout_sec: float = 60.0
) -> None:
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)

    end_time = time.time() + timeout_sec
    while not predicate():
        if time.time() >= end_time:
            raise NodeSpinTimeoutError()

        executor.spin_once(timeout_sec=0.1)

    executor.remove_node(node)
