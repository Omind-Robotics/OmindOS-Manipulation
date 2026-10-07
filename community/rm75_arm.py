"""Manufacturer RM75-B joint origins, axes, limits and link meshes, without ROS."""
import gzip
import io
import numpy as np
import basis.robot_math as rm
import basis.trimesh as trimesh
import modeling.collision_model as cm
from robot_sim._kinematics.jlchain import JLChain
from robot_sim.manipulators.manipulator_interface import ManipulatorInterface
from community.rm75_config import RM75Config
from community.waist_config import vector


def _mesh_model(spec):
    if spec['kind'] == 'box':
        mesh = trimesh.creation.box(extents=spec['size'])
    else:
        from pathlib import Path
        data = Path(spec['path']).read_bytes()
        if spec['path'].lower().endswith('.gz'):
            data = gzip.decompress(data)
        mesh = trimesh.load(io.BytesIO(data), file_type='stl')
        mesh.vertices = mesh.vertices * np.asarray(spec['scale'])
    mesh.vertices = mesh.vertices @ rm.rotmat_from_euler(*spec['rpy']).T + np.asarray(spec['xyz'])
    # Mesh visuals with conservative local bounding boxes for fast sampled planning.
    return cm.CollisionModel(mesh, cdprimit_type='box', expand_radius=.001)


class RM75Arm(ManipulatorInterface):
    def __init__(self, config, name):
        if not isinstance(config, RM75Config):
            raise ValueError('A validated RM75Config is required')
        super().__init__(name=name)
        self.config = config
        self.jlc = JLChain(homeconf=np.array(config.home_rad), name=name)
        for i, spec in enumerate(config._model_data['joints'], 1):
            joint = self.jlc.jnts[i]
            joint['loc_pos'] = np.array(spec['xyz'])
            joint['loc_rotmat'] = rm.rotmat_from_euler(*spec['rpy'])
            joint['loc_motionax'] = np.array(spec['axis'])
            joint['motion_rng'] = np.array(spec['limits'])
        for i in range(8):
            link = self.jlc.lnks[i]
            spec = config._model_data['links'][i]
            link['name'] = f'{name}_' + spec['name']
            link['collisionmodel'] = _mesh_model(spec)
            link['rgba'] = [.86, .88, .90, 1]
        # URDF link_7 is the flange. No extra reference-link rotation/length is added.
        self.jlc.reinitialize()

    def fk(self, jnt_values):
        q = np.array(vector(jnt_values, 7, 'RM75 joints'))
        limits = np.asarray(self.get_jnt_ranges())
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise ValueError('RM75 joint position exceeds manufacturer URDF limits')
        return super().fk(q)
