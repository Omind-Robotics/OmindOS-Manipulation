"""15-DoF reference robot: pitch waist + two independent seven-axis arms.

Coordinates: base x forward, y left, z up; waist axis is base +Y.
RM75-B arm data is manufacturer-sourced. Body and grippers remain reference geometry.
"""
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
import modeling.model_collection as mc
from robot_sim._kinematics.jlchain import JLChain
from robot_sim.robots.robot_interface import RobotInterface
from robot_sim.end_effectors.grippers.robotiqhe.robotiqhe import RobotiqHE
from community.seven_axis_arm import SevenAxisArm
from community.rm75_arm import RM75Arm
from community.rm75_config import RM75Config
from community.seven_axis_config import load_arm_config, SevenAxisConfig
from community.waist_config import WaistConfig, load_waist_config, vector


class MobileDualPitch(RobotInterface):
    component = 'waist_and_arms'
    dof = 15

    def __init__(self, config=None, pos=None, rotmat=None, enable_cc=True, arm_configs=None):
        self.arm_configs = load_arm_config() if arm_configs is None else dict(arm_configs)
        rm75 = all(isinstance(c, RM75Config) for c in self.arm_configs.values())
        if config is None and rm75:
            from pathlib import Path
            config = load_waist_config(Path(__file__).resolve().parents[1] / 'configs/simulation/rm75-waist.reference.json')
        self.waist_config = load_waist_config() if config is None else config
        if not isinstance(self.waist_config, WaistConfig):
            raise ValueError('config must be a validated WaistConfig')
        if set(self.arm_configs) != {'left', 'right'} or not all(
                isinstance(c, (SevenAxisConfig, RM75Config)) for c in self.arm_configs.values()):
            raise ValueError('Both validated seven-axis arm configurations are required')
        if type(self.arm_configs['left']) is not type(self.arm_configs['right']):
            raise ValueError('Both arms must use the same configuration schema')
        super().__init__(pos=np.zeros(3), rotmat=np.eye(3), name='mobile_dual_15dof')
        self.waist_pitch = self.waist_config.home_rad
        cfg = self.waist_config
        self.chassis = self._box_chain('fixed_chassis', cfg.chassis_center_m, cfg.chassis_size_m, [.3,.3,.3,1])
        self.torso = self._box_chain('pitch_torso', cfg.torso_center_m, cfg.torso_size_m, [.3,.5,.7,1])
        # Reference supports make the fixed waist pivot and shoulder mounts explicit.
        # They are simple boxes, not a model of the customer's bearing or motor housings.
        top = cfg.chassis_center_m[2] + cfg.chassis_size_m[2] / 2
        waist_height = max(.12, cfg.pivot_m[2] - top + .12)
        self.waist_support = self._box_chain('fixed_waist_support',
            [cfg.pivot_m[0], cfg.pivot_m[1], top + waist_height/2],
            [.12, .18, waist_height], [.48,.5,.54,1])
        self.shoulders = {}
        for side, key in [('lft','left'), ('rgt','right')]:
            mount = self.arm_configs[key].mount_m
            self.shoulders[side] = self._box_chain(side + '_shoulder_support',
                [mount[0]-.02, mount[1]/2, mount[2]],
                [.08, max(.08, abs(mount[1])), .08], [.48,.5,.54,1])
        self.lft_oih_infos, self.rgt_oih_infos = [], []
        for side, key in [('lft', 'left'), ('rgt', 'right')]:
            arm = (RM75Arm if rm75 else SevenAxisArm)(self.arm_configs[key], key)
            hand = RobotiqHE(enable_cc=False)
            arm.tcp_loc_pos = hand.jaw_center_pos.copy()
            arm.tcp_loc_rotmat = hand.jaw_center_rotmat.copy()
            setattr(self, side + '_arm', arm)
            setattr(self, side + '_hnd', hand)
            for suffix in ('_arm', '_hnd'):
                self.manipulator_dict[side + suffix] = arm
                self.hnd_dict[side + suffix] = hand
        self.set_base(np.zeros(3) if pos is None else pos, np.eye(3) if rotmat is None else rotmat)
        if enable_cc:
            self.enable_cc()

    @staticmethod
    def _box_chain(name, center, size, rgba):
        chain = JLChain(homeconf=np.zeros(0), name=name)
        chain.lnks[0]['collisionmodel'] = cm.gen_box(extent=size)
        chain.lnks[0]['loc_pos'] = np.array(center)
        chain.lnks[0]['rgba'] = rgba
        chain.reinitialize()
        return chain

    @staticmethod
    def _side(name):
        if name not in ('lft_arm', 'lft_hnd', 'rgt_arm', 'rgt_hnd'):
            raise ValueError('Expected lft_arm, rgt_arm, lft_hnd or rgt_hnd')
        return name[:3]

    def _hand_links(self, side):
        hand = getattr(self, side + '_hnd')
        return [hand.lft.lnks[0], hand.lft.lnks[1], hand.rgt.lnks[1]]

    def _arm_links(self, side):
        return list(getattr(self, side + '_arm').lnks[1:])

    def _arm_bases(self):
        return [getattr(self, side + '_arm').lnks[0] for side in ('lft', 'rgt')
                if isinstance(getattr(self, side + '_arm'), RM75Arm)]

    def enable_cc(self):
        if self.cc is not None:
            raise RuntimeError('Collision checking is already enabled')
        super().enable_cc()
        self.cc.add_cdlnks(self.chassis, [0])
        self.cc.add_cdlnks(self.torso, [0])
        self.cc.add_cdlnks(self.waist_support, [0])
        for shoulder in self.shoulders.values():
            self.cc.add_cdlnks(shoulder, [0])
        for side in ('lft', 'rgt'):
            arm = getattr(self, side + '_arm')
            if isinstance(arm, RM75Arm):
                self.cc.add_cdlnks(arm, [0])
        self.cc.set_active_cdlnks(self._arm_bases())
        self.cc.set_active_cdlnks([s.lnks[0] for s in self.shoulders.values()])
        body = [self.chassis.lnks[0], self.torso.lnks[0], self.waist_support.lnks[0]]
        self.cc.set_active_cdlnks(body)
        for side in ('lft', 'rgt'):
            arm, hand = getattr(self, side + '_arm'), getattr(self, side + '_hnd')
            self.cc.add_cdlnks(arm, list(range(1, 8)))
            self.cc.add_cdlnks(hand.lft, [0, 1])
            self.cc.add_cdlnks(hand.rgt, [1])
            links, fingers = self._arm_links(side), self._hand_links(side)
            self.cc.set_active_cdlnks(links + fingers)
            # Adjacent boxes share a revolute joint, so only nonadjacent links are paired.
            for i in range(5):
                self.cc.set_cdpair([links[i]], links[i+2:] + fingers)
            self.cc.set_cdpair([links[5]], fingers)
            self.cc.set_cdpair(body, links + fingers)
            # The first link shares its shoulder pivot. All later links are checked.
            self.cc.set_cdpair([self.shoulders[side].lnks[0]], links[1:] + fingers)
            if isinstance(arm, RM75Arm):
                # base/link1/link2 surround the same shoulder mechanism. The base
                # is rigidly attached to the torso/support, but checks distal links.
                self.cc.set_cdpair([arm.lnks[0]], links[2:] + fingers)
        left_base = [self.lft_arm.lnks[0]] if isinstance(self.lft_arm, RM75Arm) else []
        right_base = [self.rgt_arm.lnks[0]] if isinstance(self.rgt_arm, RM75Arm) else []
        self.cc.set_cdpair(left_base + self._arm_links('lft') + self._hand_links('lft'),
                           right_base + self._arm_links('rgt') + self._hand_links('rgt'))
        if self._arm_bases():
            self.cc.set_cdpair([self.chassis.lnks[0], self.waist_support.lnks[0]], self._arm_bases())
        self.cc.set_cdpair([self.chassis.lnks[0]],
                           [self.torso.lnks[0]] + [s.lnks[0] for s in self.shoulders.values()])
        for side, other in [('lft','rgt'), ('rgt','lft')]:
            self.cc.set_cdpair([self.shoulders[side].lnks[0]],
                               self._arm_links(other) + self._hand_links(other))
        # Re-register any remaining attachments when rebuilding after release.
        for side in ('lft', 'rgt'):
            infos = getattr(self, side + '_oih_infos')
            previous = infos[:]
            infos.clear()
            for info in previous:
                obj = info['collisionmodel']
                obj.set_pos(info['gl_pos'])
                obj.set_rotmat(info['gl_rotmat'])
                self.hold(side + '_arm', obj)

    def _update_hand(self, side):
        arm = getattr(self, side + '_arm')
        getattr(self, side + '_hnd').fix_to(arm.jnts[-1]['gl_posq'], arm.jnts[-1]['gl_rotmatq'])
        for info in getattr(self, side + '_oih_infos'):
            info['gl_pos'], info['gl_rotmat'] = arm.cvt_loc_tcp_to_gl(info['rel_pos'], info['rel_rotmat'])

    def _update_upper(self):
        pivot = np.asarray(self.waist_config.pivot_m)
        pitch = rm.rotmat_from_axangle([0, 1, 0], self.waist_pitch)
        upper_pos = self.pos + self.rotmat @ (pivot - pitch @ pivot)
        upper_rot = self.rotmat @ pitch
        self.torso.fix_to(upper_pos, upper_rot)
        for shoulder in self.shoulders.values():
            shoulder.fix_to(upper_pos, upper_rot)
        for side, key in [('lft', 'left'), ('rgt', 'right')]:
            cfg = self.arm_configs[key]
            arm = getattr(self, side + '_arm')
            arm.fix_to(upper_pos + upper_rot @ np.array(cfg.mount_m),
                       upper_rot @ rm.rotmat_from_euler(*cfg.mount_rpy))
            self._update_hand(side)

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
        self.waist_support.fix_to(self.pos, self.rotmat)
        self._update_upper()

    def fix_to(self, pos, rotmat):
        self.set_base(pos, rotmat)

    def get_jnt_values(self, component_name):
        if component_name in (self.component, 'both_arm'):
            arms = np.r_[self.lft_arm.get_jnt_values(), self.rgt_arm.get_jnt_values()]
            return np.r_[self.waist_pitch, arms] if component_name == self.component else arms
        self._side(component_name)
        return super().get_jnt_values(component_name)

    def get_jnt_ranges(self, component_name):
        if component_name in (self.component, 'both_arm'):
            arms = np.vstack((self.lft_arm.get_jnt_ranges(), self.rgt_arm.get_jnt_ranges()))
            return np.vstack((self.waist_config.limits_rad, arms)) if component_name == self.component else arms
        self._side(component_name)
        return super().get_jnt_ranges(component_name)

    def fk(self, component_name, jnt_values):
        limits = np.asarray(self.get_jnt_ranges(component_name))
        q = np.array(vector(jnt_values, len(limits), 'joint positions'))
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise ValueError('Joint position exceeds configured limits')
        # All positions are validated before mutating any component.
        if component_name == self.component:
            self.waist_pitch = q[0]
            q = q[1:]
        if component_name in (self.component, 'both_arm'):
            self.lft_arm.fk(q[:7])
            self.rgt_arm.fk(q[7:])
        else:
            self.manipulator_dict[component_name].fk(q)
        self._update_upper()

    def rand_conf(self, component_name):
        limits = np.asarray(self.get_jnt_ranges(component_name))
        return np.random.uniform(limits[:, 0], limits[:, 1])

    def get_hnd_on_manipulator(self, manipulator_name):
        return getattr(self, self._side(manipulator_name) + '_hnd')

    def hold(self, hnd_name, objcm, jaw_width=None):
        if self.cc is None:
            raise RuntimeError('Holding objects requires collision checking')
        side = self._side(hnd_name)
        other = 'rgt' if side == 'lft' else 'lft'
        if any(info['collisionmodel'] is objcm for info in self.lft_oih_infos + self.rgt_oih_infos):
            raise ValueError('Object is already held')
        if jaw_width is not None:
            self.jaw_to(hnd_name, jaw_width)
        arm = getattr(self, side + '_arm')
        rel_pos, rel_rot = arm.cvt_gl_to_loc_tcp(objcm.get_pos(), objcm.get_rotmat())
        into = ([self.chassis.lnks[0], self.torso.lnks[0], self.waist_support.lnks[0]]
                + [s.lnks[0] for s in self.shoulders.values()] + self._arm_links(side)[:-1]
                + self._arm_bases()
                + self._arm_links(other) + self._hand_links(other))
        info = self.cc.add_cdobj(objcm, rel_pos, rel_rot, into)
        getattr(self, side + '_oih_infos').append(info)
        self._update_hand(side)
        return rel_pos, rel_rot

    def release(self, hnd_name, objcm, jaw_width=None):
        side = self._side(hnd_name)
        if jaw_width is not None:
            self.jaw_to(hnd_name, jaw_width)
        infos = getattr(self, side + '_oih_infos')
        for info in infos[:]:
            if info['collisionmodel'] is objcm:
                objcm.set_pos(info['gl_pos'])
                objcm.set_rotmat(info['gl_rotmat'])
                infos.remove(info)
                # Rebuild to reclaim object masks and avoid stale node indices in the upstream checker.
                self.disable_cc()
                self.enable_cc()
                break

    def gen_meshmodel(self, **kwargs):
        result = mc.ModelCollection(name=self.name)
        for chain in (self.chassis, self.waist_support, self.torso, *self.shoulders.values(),
                      self.lft_arm, self.rgt_arm, self.lft_hnd, self.rgt_hnd):
            chain.gen_meshmodel(**kwargs).attach_to(result)
        for info in self.lft_oih_infos + self.rgt_oih_infos:
            obj = info['collisionmodel'].copy()
            obj.set_pos(info['gl_pos'])
            obj.set_rotmat(info['gl_rotmat'])
            obj.attach_to(result)
        return result
