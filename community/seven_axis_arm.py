"""Seven revolute joints with configurable reference box links; no UR3e substitution."""
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
from robot_sim._kinematics.jlchain import JLChain
from robot_sim.manipulators.manipulator_interface import ManipulatorInterface
from community.seven_axis_config import SevenAxisConfig
from community.waist_config import vector


class SevenAxisArm(ManipulatorInterface):
    def __init__(self, config, name):
        if not isinstance(config, SevenAxisConfig):
            raise ValueError('A validated SevenAxisConfig is required')
        super().__init__(name=name)
        self.config = config
        self.jlc = JLChain(homeconf=np.array(config.home_rad), name=name)
        for i in range(1, 8):
            joint = self.jlc.jnts[i]
            joint['loc_pos'] = np.array([config.lengths_m[i-2], 0, 0]) if i > 1 else np.zeros(3)
            joint['loc_motionax'] = np.array(config.axes[i-1])
            joint['motion_rng'] = np.array(config.limits_rad[i-1])
            link = self.jlc.lnks[i]
            link['name'] = f'{name}_link_{i}'
            link['loc_pos'] = np.array([config.lengths_m[i-1] / 2, 0, 0])
            link['collisionmodel'] = cm.gen_box(extent=[config.lengths_m[i-1],
                                                       config.link_width_m, config.link_width_m])
            link['rgba'] = [.20, .52, .80, 1] if i % 2 else [.75, .78, .82, 1]
        self.jlc.jnts[-1]['loc_pos'] = np.array([config.lengths_m[-1], 0, 0])
        # Reference gripper's local +Z points along the final link's +X.
        self.jlc.jnts[-1]['loc_rotmat'] = rm.rotmat_from_euler(0, np.pi/2, 0)
        self.jlc.reinitialize()

    def fk(self, jnt_values):
        q = np.array(vector(jnt_values, 7, 'seven-axis joints'))
        limits = np.asarray(self.get_jnt_ranges())
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise ValueError('Seven-axis joint position exceeds configured limits')
        return super().fk(q)
