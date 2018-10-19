import unittest
from simpleagent import SimpleAgent

class SimpleAgentTest(unittest.TestCase):

    def test_path_planA(self):

        a = SimpleAgent()
        expected = ['A']

        actual = a.path_plan('A','A')

        self.assertListEqual(expected, actual)

    def test_path_planAB(self):

        a = SimpleAgent()
        expected = ['A','B']

        actual = a.path_plan('A','B')

        self.assertListEqual(expected, actual)

    def test_path_planAC(self):

        a = SimpleAgent()
        expected = ['A','C']

        actual = a.path_plan('A','C')

        self.assertListEqual(expected, actual)

    def test_path_planAD(self):

        a = SimpleAgent()
        expected = ['A','B','D']

        actual = a.path_plan('A','D')

        self.assertListEqual(expected, actual)

    def test_path_planB(self):

        a = SimpleAgent()
        expected = ['B']

        actual = a.path_plan('B','B')

        self.assertListEqual(expected, actual)

    def test_path_planBA(self):

        a = SimpleAgent()
        expected = ['B','A']

        actual = a.path_plan('B','A')

        self.assertListEqual(expected, actual)

    def test_path_planBC(self):

        a = SimpleAgent()
        expected = ['B','A','C']

        actual = a.path_plan('B','C')

        self.assertListEqual(expected, actual)

    def test_path_planBD(self):

        a = SimpleAgent()
        expected = ['B','D']

        actual = a.path_plan('B','D')

        self.assertListEqual(expected, actual)

if __name__ == '__main__':
    unittest.main()
