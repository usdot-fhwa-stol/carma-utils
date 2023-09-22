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

from lifecycle_msgs.srv import ChangeState

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from .exceptions import WaitForServiceTimeoutError, ServiceCallFailedError


def transition_node(node_name: str, transition_id: int, ros_context: Context) -> None:
    transitioner = rclpy.create_node("transitioner", context=ros_context)

    srv_client = transitioner.create_client(ChangeState, f"{node_name}/change_state")

    if not srv_client.wait_for_service(timeout_sec=1.0):
        raise WaitForServiceTimeoutError(
            f"Service `{node_name}/change_state not ready within "
            "specified timeout of `1.0` seconds"
        )

    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(transitioner)

    request = ChangeState.Request()
    request.transition.id = transition_id

    future = srv_client.call_async(request)
    executor.spin_once_until_future_complete(future)

    executor.remove_node(transitioner)
    transitioner.destroy_node()

    if not future.result().success:
        raise ServiceCallFailedError(
            f"Failed to transition lifecycle node `{node_name}`"
        )
