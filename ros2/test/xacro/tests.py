import unittest
from xml.etree import ElementTree


class TestXacro(unittest.TestCase):

    def _check(self, actual_xml_file_name: str, expected_xml: str):
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
        self._check('ros2/test/xacro/test_urdf.urdf', expected_xml)

    def test_include(self):
        expected_xml = """
<xml>
  <a foo="1 1.0"/>
</xml>
"""
        self._check('ros2/test/xacro/test_include.urdf', expected_xml)

    def test_with_params(self):
        expected_xml = """
<xml>
  <a foo="bar"/>
</xml>
"""
        self._check('ros2/test/xacro/test_with_params.urdf', expected_xml)


if __name__ == '__main__':
    unittest.main()
