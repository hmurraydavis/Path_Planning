import unittest
import astar

class Test(unittest.TestCase):
	def testLoadscsv(self):
		d = astar.Data()
		self.assertGreater(len(d.data),0)

	def testclosedset(self):
		d = astar.Data()
		self.assertEqual(len(d.closedset),0)

if __name__ == '__main__':
	unittest.main()