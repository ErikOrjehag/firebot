
import rclpy
import rclpy.node

import PySimpleGUI as sg
import typing as tp
import std_msgs.msg

def create_led_indicator(key: str = "None", max_radius: int = 14) -> sg.Graph:
    return sg.Graph(canvas_size=(max_radius, max_radius),
                    graph_bottom_left=(-max_radius, -max_radius),
                    graph_top_right=(max_radius, max_radius),
                    pad=(0, 0), key=key)

def update_led(
    window: sg.Window, key: str, value: tp.Optional[bool],
    true_color: str = 'green', false_color: str = 'red', none_color: str = 'pink',
) -> None:
    if value is None:
        set_led(window, key, none_color)
    elif value:
        set_led(window, key, true_color)
    else:
        set_led(window, key, false_color)

def set_led(window, key: str, color: str, radius: int = 10) -> None:
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), radius, fill_color=color, line_color=color)

class InputNode:

    def __init__(self, node: rclpy.node.Node):
        self.node = node
    
        self.start_pub = self.node.create_publisher(
            std_msgs.msg.Empty, "/start", 1)
        self.green_pub = self.node.create_publisher(
            std_msgs.msg.Bool, "/btn/green", 1)
        self.red_pub = self.node.create_publisher(
            std_msgs.msg.Bool, "/btn/red", 1)
        
        node.create_subscription(std_msgs.msg.Bool, '/led/green', self.led_green_cb, 10)
        node.create_subscription(std_msgs.msg.Bool, '/led/yellow', self.led_yellow_cb, 10)
        node.create_subscription(std_msgs.msg.Bool, '/led/red', self.led_red_cb, 10)
        node.create_subscription(std_msgs.msg.Bool, '/buzz', self.led_buzz_cb, 10)
        node.create_subscription(std_msgs.msg.Bool, '/snuff', self.led_snuff_cb, 10)

    def led_green_cb(self, msg: std_msgs.msg.Bool):
        update_led(self.window, 'led_green', msg.data,
               true_color='green', false_color='black')

    def led_yellow_cb(self, msg: std_msgs.msg.Bool):
        update_led(self.window, 'led_yellow', msg.data,
               true_color='yellow', false_color='black')

    def led_red_cb(self, msg: std_msgs.msg.Bool):
        update_led(self.window, 'led_red', msg.data,
               true_color='red', false_color='black')

    def led_buzz_cb(self, msg: std_msgs.msg.Bool):
        update_led(self.window, 'led_buzz', msg.data,
               true_color='white', false_color='black')

    def led_snuff_cb(self, msg: std_msgs.msg.Bool):
        update_led(self.window, 'led_snuff', msg.data,
               true_color='orange', false_color='black')

    def create_layout(self):
        return [[sg.Button("Start sound", key="btn_start")],
                [sg.Checkbox("Green button", enable_events=True, key="btn_green")],
                [sg.Checkbox("Red button", enable_events=True, key="btn_red")],
                [sg.Text("Green LED"), create_led_indicator('led_green')],
                [sg.Text("Yellow LED"), create_led_indicator('led_yellow')],
                [sg.Text("Red LED"), create_led_indicator('led_red')],
                [sg.Text("Buzz LED"), create_led_indicator('led_buzz')],
                [sg.Text("Snuff LED"), create_led_indicator('led_snuff')],
                ]

    def spin_with_gui(self):
        sg.theme("Dark Grey 13")
        sg.set_options(element_padding=(5, 5), )
        layout = self.create_layout()
        self.window = sg.Window("Input sim", layout, finalize=True)


        set_led(self.window, "led_green", "black")
        set_led(self.window, "led_yellow", "black")
        set_led(self.window, "led_red", "black")
        set_led(self.window, "led_buzz", "black")
        set_led(self.window, "led_snuff", "black")

        while rclpy.ok():
            event, values = self.window.read(timeout=0.01)

            if event != "__TIMEOUT__":
                print(event, values)

            if event == sg.WIN_CLOSED:
                break

            if event == "btn_start":
                self.start_pub.publish(std_msgs.msg.Empty())
            if event == "btn_green":
                self.green_pub.publish(std_msgs.msg.Bool(data=values["btn_green"]))
            if event == "btn_red":
                self.red_pub.publish(std_msgs.msg.Bool(data=values["btn_red"]))

            rclpy.spin_once(self.node, timeout_sec=0.001)
        self.window.close()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("input_node")
    input_node = InputNode(node)
    
    try:
        input_node.spin_with_gui()
    except KeyboardInterrupt:
        pass

    input_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()