import unittest
from xml.etree import ElementTree
import os


class TestXacro(unittest.TestCase):

    def _get_path(self, path):
        if os.path.exists(path):
            return path
        # Try finding it in external repository
        external_path = os.path.join("..", "rules_ros2+", path)
        if os.path.exists(external_path):
            return external_path
        # Fallback for debugging (though we removed debug prints)
        return path

    def _check(self, actual_xml_file_name: str, expected_xml: str):
        actual_xml_file_name = self._get_path(actual_xml_file_name)
        actual = ElementTree.tostring(
            ElementTree.parse(actual_xml_file_name).getroot(),
            encoding='unicode',
            method='xml')
        expected = ElementTree.tostring(ElementTree.fromstring(expected_xml),
                                        encoding='unicode',
                                        method='xml')
        self.assertEqual(actual, expected)

    def test_urdf(self):
        expected_xml = """
<xml>
  <a foo="1 1.0"/>
  <b bar="2 2.0"/>
</xml>
"""
        self._check('ros2/test/xacro/test_urdf/model.urdf', expected_xml)

    def test_include(self):
        expected_xml = """
<xml>
  <a foo="1 1.0"/>
</xml>
"""
        self._check('ros2/test/xacro/test_include/model.urdf', expected_xml)

    def test_with_params(self):
        expected_xml = """
<xml>
  <a foo="bar"/>
</xml>
"""
        self._check('ros2/test/xacro/test_with_params/model.urdf', expected_xml)


if __name__ == '__main__':
    unittest.main()
