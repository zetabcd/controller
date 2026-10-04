#!/usr/bin/env python3
"""Interactive selection outside the control process: c enters CMD in simulation; numbers/r plan; e executes; f finishes."""
import queue
import threading
import rclpy
from rclpy.node import Node
from gap_msgs.srv import PlanGaps
from gap_msgs.msg import ExecutionStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Console(Node):
    def __init__(self):
        super().__init__('gap_console')
        self.planner = self.create_client(PlanGaps, '/gap/plan')
        self.start = self.create_client(Trigger, '/gap/start')
        self.enter = self.create_client(Trigger, '/gap/enter_cmd')
        self.finish = self.create_client(Trigger, '/gap/finish')
        self.last_count = None
        self.commands = queue.Queue()
        self.last_state = None
        self.create_subscription(ExecutionStatus, '/gap/execution', self.status, 10)
        self.create_subscription(String, '/gap/planner_status', lambda m: print(m.data, flush=True), 10)
        self.create_timer(0.1, self.command)
        threading.Thread(target=self.read_input, daemon=True).start()
        print('先进入 CMD：仿真输入 c，实机 AUX2 UP。随后输入数量选择本次窗框并规划；e 执行；r 同数量反向继续；f 结束任务回 AUTO_HOVER；q 仅退出控制台。', flush=True)

    def status(self, message):
        key = (message.state, message.reason)
        if key != self.last_state:
            self.last_state = key
            print(f'{message.state}: {message.reason}', flush=True)

    def read_input(self):
        try:
            while True:
                self.commands.put(input().strip())
        except EOFError:
            self.commands.put('q')

    def command(self):
        if self.commands.empty():
            return
        text = self.commands.get()
        if text == 'q':
            rclpy.shutdown()
            return
        if text == 'r' and self.last_count is not None:
            text = str(self.last_count)
        if text.isdecimal() and 0 < int(text) <= 10:
            client, request = self.planner, PlanGaps.Request(count=int(text))
        elif text == 'c':
            client, request = self.enter, Trigger.Request()
        elif text == 'f':
            client, request = self.finish, Trigger.Request()
        elif text == 'e':
            client, request = self.start, Trigger.Request()
        else:
            print('请输入 1..10、c、e、r、f 或 q', flush=True)
            return
        if not client.service_is_ready():
            print('服务未就绪；实机用 AUX2 UP 进入 CMD 后，再选择数量和 e 执行。', flush=True)
            return
        def response(future):
            try:
                result = future.result()
                if client is self.planner and result.accepted:
                    self.last_count = request.count
                print(result.message, flush=True)
            except Exception as error:
                print(f'服务调用失败：{error}', flush=True)
        client.call_async(request).add_done_callback(response)


def main():
    rclpy.init()
    node = Console()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
