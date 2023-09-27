# CARMA launch testing

This package provides Python classes and functions to supplement the ROS 2
[`launch_testing`][launch_testing_ros_index_link] package.

[launch_testing_ros_index_link]: https://index.ros.org/p/launch_testing/

## Getting started

The `launch_testing` framework builds on top of the ROS 2 Launch system, so
launch tests are simple Launch files with some extra functionality. They
combine launch files with Python's `unittest` library.

As an example, let's create a launch test for the `demo_nodes_cpp` package's
`talker` Node. First, we need to create a test harness that will listen for
messages the `talker` Node publishes:

```python
class TestHarnessNode(rclpy.node.Node):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__("test_harness", *args, **kwargs)

        self.chatter_sub = self.create_subscription(
            String,
            "/chatter",
            lambda msg: self.heard_msgs.append(msg),
            1,
        )

        self.heard_msgs = []
```

Here we create a subscription to for the `/chatter` topic, which is where the
`talker` Node publishes. Next, we create a `list` to contain received messages.
We'll see later how the test harness integrates with the `talker`.

Next, we need to generate the `LaunchDescription` that will spawn all of our
ROS components:

```python
@pytest.mark.launch_test
def generate_test_description():
    node_under_test = launch_ros.actions.Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="node_under_test",
    )

    launch_description = LaunchDescription(
        [node_under_test, TimerAction(period=1.0, actions=[ReadyToTest()])]
    )

    return launch_description
```

This should resemble a `LaunchDescription` from a normal `*_launch.py` or
`*.launch.py` file. Two differences to note are the `@pytest` decorator and the
`TimerAction`. The function decorator lets the `launch_testing` framework know
that this launch description is for a test case. The `TimerAction` object will
delay the launch test for one second (a configurable argument) to allow the
`talker` Node time to start up. The test will begin after the delay.

Now we need to specify the actual test we want to do. This is where Python's
`unittest` library comes in. In this example, we are testing to check that the
`talker` Node publishes the expected `Hello World` messages:

```python
class TestTalker(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()

        cls.context = Context()

        rclpy.init(context=cls.context)
        cls.test_harness_node = TestHarnessNode(context=cls.context)

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()

        rclpy.shutdown(context=cls.context)

    def test_talker(self):
        spin_node_until(
            self.test_harness_node,
            LenIncreases(self.test_harness_node.heard_msgs),
            self.context,
        )

        self.assertGreaterEqual(len(self.test_harness_node.heard_msgs), 0)

        latest_msg = self.test_harness_node.heard_msgs[-1]
        self.assertTrue("Hello World" in latest_msg.data)
```

The `setUpClass()` and `tearDownClass()` functions handle starting and stopping
ROS, respectively. The `setUpClass()` function additionally creates an instance
of out test harness that we will use in our `test_talker()` test case.

Now for the main part of the launch test: the test itself. Our example test
case will first spin the test harness until it receives a message or times out
(the latter scenario will `raise` an error). The `LenIncrease` predicate is a
`callable` object that returns `True` when the condition is satisfied. See the
[`carma_launch_testing.predicates`][predicates_module_link] for more predicate
classes.

Assuming the `spin_node_until()` function did not timeout, the test harness
should have a new message in its `heard_msgs` list. We can grab the most
recent one and assert that its content is as we expect.

Sometimes we may want to check that our node under test exits gracefully. The
`launch testing` package provides post-shutdown tests to check various
conditions after the main tests complete. For our example, we will check that
the `talker` Node exited with a non-error exit code:

```python
@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
```

Post-shutdown tests use the `unittest` library but have the
`@post_shutdown_test` class decorator. This tells `launch_testing` that the
contained tests should run only after the launched Nodes shutdown.

Putting all the pieces together, we get the final launch test configuration:

```python
import unittest

from carma_launch_testing.predicates import LenIncreases
from carma_launch_testing.spinning import spin_node_until

from std_msgs.msg import String

import launch_ros.actions

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes

import pytest

import rclpy
from rclpy.context import Context
import rclpy.node


class TestHarnessNode(rclpy.node.Node):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__("test_harness", *args, **kwargs)

        self.chatter_sub = self.create_subscription(
            String,
            "/chatter",
            lambda msg: self.heard_msgs.append(msg),
            1,
        )

        self.heard_msgs = []


@pytest.mark.launch_test
def generate_test_description():
    node_under_test = launch_ros.actions.Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="node_under_test",
    )

    launch_description = LaunchDescription(
        [node_under_test, TimerAction(period=1.0, actions=[ReadyToTest()])]
    )

    return launch_description


class TestTalker(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()

        cls.context = Context()

        rclpy.init(context=cls.context)
        cls.test_harness_node = TestHarnessNode(context=cls.context)

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()

        rclpy.shutdown(context=cls.context)

    def test_talker(self):
        spin_node_until(
            self.test_harness_node,
            LenIncreases(self.test_harness_node.heard_msgs),
            self.context,
        )

        self.assertGreaterEqual(len(self.test_harness_node.heard_msgs), 0)

        latest_msg = self.test_harness_node.heard_msgs[-1]
        self.assertTrue("Hello World" in latest_msg.data)


@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
```

Congratulations! You have created your first launch test.

[predicates_module_link]: carma_launch_testing/predicates.py

> [!IMPORTANT]\
> Each test case defined for the launch test will run concurrently. Keep this
> in mind if you plan on testing sequence-dependent behaviors.

### Going further

The above example is only a brief overview to launch testing. Tests can be as
simple or complicated as you wish. Check out some of the CARMA Platform packages
to see real-world launch tests.
