"""Strict local URDF loader for the release's seven-revolute-joint arm topology.

Unsupported topology or geometry is rejected rather than silently replaced by
RM75 geometry. Meshes are resolved locally; no ROS or network resolver is used.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def numbers(text, count, name):
    try:
        values = tuple(float(x) for x in text.split())
        if len(values) != count or not all(math.isfinite(x) for x in values):
            raise ValueError
        return values
    except (ValueError, AttributeError) as exc:
        raise ValueError(f'Invalid URDF {name}: expected {count} finite numbers') from exc


def origin(element):
    node = element.find('origin')
    return (numbers(node.get('xyz', '0 0 0'), 3, 'origin xyz'),
            numbers(node.get('rpy', '0 0 0'), 3, 'origin rpy')) if node is not None else ((0.,)*3, (0.,)*3)


def load_model(path):
    path = Path(path).resolve(strict=True)
    root = ET.parse(path).getroot()
    if root.tag != 'robot':
        raise ValueError('URDF root must be robot')
    links = {node.get('name'): node for node in root.findall('link')}
    joints = root.findall('joint')
    if len(links) != 8 or len(root.findall('link')) != 8 or len(joints) != 7:
        raise ValueError('Supported arm URDF requires exactly eight links and seven revolute joints')
    edges, children, names = {}, set(), set()
    for joint in joints:
        if joint.get('type') != 'revolute' or joint.find('mimic') is not None:
            raise ValueError('Only independent revolute joints are supported')
        try:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            name = joint.get('name')
            if (parent not in links or child not in links or parent in edges
                    or child in children or not name or name in names):
                raise ValueError('URDF must be one unbranched chain with unique joint names')
            xyz, rpy = origin(joint)
            axis_node = joint.find('axis')
            axis = numbers(axis_node.get('xyz') if axis_node is not None else '1 0 0', 3, 'axis')
            if abs(sum(x*x for x in axis)-1.) > 1e-8:
                raise ValueError('URDF joint axes must be unit vectors')
            limit = joint.find('limit')
            bounds = numbers(f"{limit.get('lower')} {limit.get('upper')}", 2, 'joint limits')
            if bounds[0] >= bounds[1]:
                raise ValueError('URDF joint limits must be ordered')
        except AttributeError as exc:
            raise ValueError('URDF joint requires parent, child and position limits') from exc
        edges[parent] = dict(name=name, child=child, xyz=xyz, rpy=rpy, axis=axis, limits=bounds)
        children.add(child)
        names.add(name)
    roots = set(links)-children
    if len(roots) != 1:
        raise ValueError('URDF must have exactly one root link')
    ordered_links, ordered_joints = [], []
    current = roots.pop()
    while current not in ordered_links:
        ordered_links.append(current)
        if current not in edges:
            break
        ordered_joints.append(edges[current])
        current = edges[current]['child']
    if len(ordered_links) != 8 or len(ordered_joints) != 7:
        raise ValueError('URDF has a disconnected or cyclic chain')
    geometries = []
    for name in ordered_links:
        collisions = links[name].findall('collision')
        if len(collisions) != 1:
            raise ValueError(f'{name}: exactly one collision geometry is required')
        collision = collisions[0]
        geometry = collision.find('geometry')
        if geometry is None or len(geometry) != 1:
            raise ValueError(f'{name}: missing or ambiguous collision geometry')
        shape = geometry[0]
        xyz, rpy = origin(collision)
        spec = dict(name=name, xyz=xyz, rpy=rpy, kind=shape.tag)
        if shape.tag == 'mesh':
            filename = shape.get('filename', '')
            if filename.startswith('package://'):
                parts = filename[len('package://'):].split('/', 1)
                if len(parts) != 2:
                    raise ValueError('Invalid package mesh path')
                filename = parts[1]
            elif '://' in filename:
                raise ValueError('Only local mesh paths and package:// paths are supported')
            mesh = path.parent / filename
            if not mesh.is_file() and Path(str(mesh)+'.gz').is_file():
                mesh = Path(str(mesh)+'.gz')
            if not mesh.is_file() or not (str(mesh).lower().endswith('.stl') or str(mesh).lower().endswith('.stl.gz')):
                raise ValueError(f'{name}: missing local STL mesh: {mesh}')
            scale = numbers(shape.get('scale', '1 1 1'), 3, 'mesh scale')
            if min(scale) <= 0:
                raise ValueError('URDF mesh scale must be positive')
            spec.update(path=str(mesh.resolve()), scale=scale)
        elif shape.tag == 'box':
            size = numbers(shape.get('size'), 3, 'box size')
            if min(size) <= 0:
                raise ValueError('URDF box dimensions must be positive')
            spec['size'] = size
        else:
            raise ValueError(f'Unsupported URDF collision geometry: {shape.tag}')
        geometries.append(spec)
    return dict(joints=ordered_joints, links=geometries, urdf=str(path))
