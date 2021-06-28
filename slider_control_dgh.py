import time
import numpy as np

import dynamic_graph_manager_cpp_bindings
from robot_properties_solo.solo8wrapper import Solo8Config
from dynamic_graph_head import ThreadHead, Vicon, HoldPDController


class FootSliderController:
    def __init__(self, head, Kp, Kd, with_sliders=False):
        self.head = head
        self.Kp = Kp
        self.Kd = Kd

        self.slider_scale = np.pi
        self.with_sliders = with_sliders

        self.joint_positions = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor('joint_velocities')

        if with_sliders:
            self.slider_positions = head.get_sensor('slider_positions')

    def warmup(self, thread_head):
        self.zero_pos = self.joint_positions.copy()

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def go_zero(self):
        # TODO: Make this an interpolation.
        self.zero_pos = np.zeros_like(self.zero_pos)

        if self.with_sliders:
            self.slider_zero_pos = self.map_sliders(self.slider_positions)

    def map_sliders(self, sliders):
        sliders_out = np.zeros_like(self.joint_positions)
        if self.joint_positions.shape[0] == 8:
            slider_A = sliders[0]
            slider_B = sliders[1]
            for i in range(4):
                sliders_out[2 * i + 0] = slider_A
                sliders_out[2 * i + 1] = 2. * (1. - slider_B)

                if i >= 2:
                    sliders_out[2 * i + 0] *= -1
                    sliders_out[2 * i + 1] *= -1

        return sliders_out

    def run(self, thread_head):
        def get_vicon(name1, name2=None):
            if name2 is None:
                name2 = name1
            pos, vel = thread_head.vicon.get_state(name1 + '/' + name2)
            return np.hstack([pos, vel])

        self.vicon_solo = get_vicon('solo8v2')
        self.vicon_leg_fr = get_vicon('solo8_fr', 'hopper_foot')
        self.vicon_leg_hl = get_vicon('solo8_hl', 'hopper_foot')
        self.vicon_leg_hr = get_vicon('solo8_hr', 'hopper_foot')

        if self.with_sliders:
            self.des_position = self.slider_scale * (
                self.map_sliders(self.slider_positions) - self.slider_zero_pos
                ) + self.zero_pos 
        else:
            self.des_position = self.zero_pos

        ## todo; hit ground to measure latency

        self.tau = self.Kp * (self.des_position - self.joint_positions) - self.Kd * self.joint_velocities
        thread_head.head.set_control('ctrl_joint_torques', self.tau)


if __name__ == "__main__":
    ###
    # Create the dgm communication and instantiate the controllers.
    head = dynamic_graph_manager_cpp_bindings.DGMHead(Solo8Config.dgm_yaml_path)

    # Create the controllers.
    hold_pd_controller = HoldPDController(head, 3., 0.05, with_sliders=True)

    foot_slider_controller = FootSliderController(head, 3., 0.05, with_sliders=True)

    thread_head = ThreadHead(
        0.001,
        hold_pd_controller,
        head,
        [
            ('vicon', Vicon('172.24.117.119:801', [
                'solo8v2/solo8v2',
                'solo8_fr/hopper_foot',
                'solo8_hl/hopper_foot',
                'solo8_hr/hopper_foot'
            ]))
        ]
    )

    # Start the parallel processing.
    thread_head.start()

    time.sleep(0.1)

    thread_head.switch_controllers(foot_slider_controller)
    thread_head.start_logging()

    time.sleep(3)
    thread_head.stop_logging()