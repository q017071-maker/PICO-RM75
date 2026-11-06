import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = PROJECT_ROOT / 'Assets' / 'ThirdParty' / 'RealMan' / 'RM75_6FB'
SOURCE_ROOT = MODEL_ROOT / 'Source'
MESH_ROOT = SOURCE_ROOT / 'Meshes'
EXPECTED_MESHES = {
    'base_link.STL', 'link_1.STL', 'link_2.STL', 'link_3.STL',
    'link_4.STL', 'link_5.STL', 'link_6.STL', 'link_7.STL',
}
PINNED_COMMIT = '9d37f52605aa00c7b0ed8143e5e46b8ffdb9317a'


class Rm75ModelAssetTests(unittest.TestCase):
    def test_official_source_and_license_are_recorded(self):
        source = (MODEL_ROOT / 'SOURCE.md').read_text(encoding='utf-8')
        license_text = (MODEL_ROOT / 'LICENSE').read_text(encoding='utf-8')
        self.assertIn('RealManRobot/rm_models', source)
        self.assertIn(PINNED_COMMIT, source)
        self.assertIn('Apache License, Version 2.0', license_text)

    def test_urdf_describes_seven_revolute_joints_and_eight_meshes(self):
        root = ET.parse(SOURCE_ROOT / 'RM75-6FB.urdf').getroot()
        joints = root.findall('joint')
        self.assertEqual(len(joints), 7)
        self.assertEqual([joint.attrib['name'] for joint in joints],
                         [f'joint_{index}' for index in range(1, 8)])
        self.assertTrue(all(joint.attrib.get('type') == 'revolute'
                            for joint in joints))
        references = {
            Path(mesh.attrib['filename']).name
            for mesh in root.findall('./link/visual/geometry/mesh')
        }
        self.assertEqual(references, EXPECTED_MESHES)

    def test_binary_stl_geometry_is_finite_and_meter_scaled(self):
        self.assertEqual({path.name for path in MESH_ROOT.glob('*.STL')},
                         EXPECTED_MESHES)
        for mesh_path in sorted(MESH_ROOT.glob('*.STL')):
            with self.subTest(mesh=mesh_path.name):
                payload = mesh_path.read_bytes()
                self.assertGreaterEqual(len(payload), 84)
                triangle_count = struct.unpack_from('<I', payload, 80)[0]
                self.assertGreater(triangle_count, 0)
                self.assertEqual(len(payload), 84 + triangle_count * 50)
                minima = [math.inf] * 3
                maxima = [-math.inf] * 3
                for triangle in range(triangle_count):
                    offset = 84 + triangle * 50
                    values = struct.unpack_from('<12fH', payload, offset)
                    self.assertTrue(all(math.isfinite(value)
                                        for value in values[:12]))
                    for vertex in range(3):
                        xyz = values[3 + vertex * 3:6 + vertex * 3]
                        minima = [min(a, b) for a, b in zip(minima, xyz)]
                        maxima = [max(a, b) for a, b in zip(maxima, xyz)]
                extent = [high - low for low, high in zip(minima, maxima)]
                self.assertGreater(max(extent), 0.001)
                self.assertLess(max(extent), 1.0)


if __name__ == '__main__':
    unittest.main()
