"""Legacy 13-joint pitch waist plus two UR3e arms with reference collision boxes.

Coordinates: base x forward, y left, z up. The waist rotates about base +Y.
No hardware controller, dynamics or calibrated customer geometry is provided.
"""
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
from robot_sim._kinematics.jlchain import JLChain
from robot_sim.robots.ur3e_dual.ur3e_dual import UR3EDual
from community.waist_config import WaistConfig, load_waist_config, vector


class MobileDualPitch(UR3EDual):
    component = 'waist_and_arms'

    def __init__(self, config=None, pos=None, rotmat=None, enable_cc=True):
        self.waist_config = load_waist_config() if config is None else config
        if not isinstance(self.waist_config, WaistConfig):
            raise ValueError('config must be a validated WaistConfig')
        self.waist_pitch = self.waist_config.home_rad
        super().__init__(pos=np.zeros(3) if pos is None else pos,
                         rotmat=np.eye(3) if rotmat is None else rotmat, enable_cc=False)
        cfg = self.waist_config
        # Replace the upstream monolithic body mesh: pitching it would tilt the chassis.
        self.lft_body.lnks[0]['collisionmodel'] = cm.gen_box(extent=cfg.torso_size_m)
        self.lft_body.lnks[0]['loc_pos'] = np.array(cfg.torso_center_m)
        self.lft_body.lnks[0]['rgba'] = [.3, .5, .7, 1]
        self.lft_body.reinitialize()
        self.chassis = JLChain(pos=self.pos, rotmat=self.rotmat, homeconf=np.zeros(0), name='fixed_chassis')
        self.chassis.lnks[0]['collisionmodel'] = cm.gen_box(extent=cfg.chassis_size_m)
        self.chassis.lnks[0]['loc_pos'] = np.array(cfg.chassis_center_m)
        self.chassis.lnks[0]['rgba'] = [.3, .3, .3, 1]
        self.chassis.reinitialize()
        self.set_base(self.pos, self.rotmat)
        if enable_cc:
            self.enable_cc()

    def enable_cc(self):
        super().enable_cc()
        upper = list(self.cc.all_cdelements)
        self.cc.add_cdlnks(self.chassis, [0])
        self.cc.set_active_cdlnks([self.chassis.lnks[0]])
        self.cc.set_cdpair([self.chassis.lnks[0]], upper)

    def _update_upper(self):
        pivot = np.asarray(self.waist_config.pivot_m)
        pitch = rm.rotmat_from_axangle([0, 1, 0], self.waist_pitch)
        upper_pos = self.pos + self.rotmat @ (pivot - pitch @ pivot)
        upper_rot = self.rotmat @ pitch
        for side in ('lft', 'rgt'):
            body, arm = getattr(self, side + '_body'), getattr(self, side + '_arm')
            q = arm.get_jnt_values().copy()
            body.fix_to(upper_pos, upper_rot)
            arm.fix_to(body.jnts[-1]['gl_posq'], body.jnts[-1]['gl_rotmatq'])
            super().fk(side + '_arm', q)  # Also updates grippers and held-object poses.

    def set_waist_pitch(self, angle):
        angle = vector([angle], 1, 'waist_pitch')[0]
        if not self.waist_config.limits_rad[0] <= angle <= self.waist_config.limits_rad[1]:
            raise ValueError('Waist pitch exceeds configured limits')
        self.waist_pitch = angle
        self._update_upper()

    def set_base(self, pos, rotmat):
        position = np.asarray(vector(pos, 3, 'base position'))
        rotation = np.asarray(rotmat, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8)):
            raise ValueError('Base orientation must be a rotation matrix')
        self.pos, self.rotmat = position, rotation.copy()
        self.chassis.fix_to(self.pos, self.rotmat)
        self._update_upper()

    def fix_to(self, pos, rotmat):
        self.set_base(pos, rotmat)

    def get_jnt_values(self, component_name):
        if component_name in (self.component, 'both_arm'):
            arms = np.r_[self.lft_arm.get_jnt_values(), self.rgt_arm.get_jnt_values()]
            return np.r_[self.waist_pitch, arms] if component_name == self.component else arms
        return super().get_jnt_values(component_name)

    def get_jnt_ranges(self, component_name):
        if component_name in (self.component, 'both_arm'):
            arms = np.vstack((self.lft_arm.get_jnt_ranges(), self.rgt_arm.get_jnt_ranges()))
            return np.vstack((self.waist_config.limits_rad, arms)) if component_name == self.component else arms
        return super().get_jnt_ranges(component_name)

    def fk(self, component_name, jnt_values):
        if component_name == self.component:
            q = np.asarray(vector(jnt_values, 13, 'waist and arm joints'))
            limits = self.get_jnt_ranges(component_name)
            if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
                raise ValueError('Joint position exceeds configured limits')
            self.set_waist_pitch(q[0])
            super().fk('both_arm', q[1:])
        else:
            super().fk(component_name, jnt_values)

    def rand_conf(self, component_name):
        if component_name == self.component:
            limits = self.get_jnt_ranges(component_name)
            return np.random.uniform(limits[:, 0], limits[:, 1])
        return super().rand_conf(component_name)

    def hold(self, hnd_name, objcm, jaw_width=None):
        result = super().hold(hnd_name, objcm, jaw_width)
        infos = self.lft_oih_infos if hnd_name in ('lft_arm', 'lft_hnd') else self.rgt_oih_infos
        # The upstream attachment pairs only know about the original upper body.
        self.cc.set_cdpair([infos[-1]], [self.chassis.lnks[0]])
        infos[-1]['intolist'].append(self.chassis.lnks[0])
        return result

    def gen_meshmodel(self, **kwargs):
        result = super().gen_meshmodel(**kwargs)
        self.chassis.gen_meshmodel(toggle_tcpcs=False).attach_to(result)
        return result
