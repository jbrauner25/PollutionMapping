import gridandcell
import unittest


class TestMethods(unittest.TestCase):

    def test_upper(self):
        cell1 = gridandcell.Cell(0,0,5.0,5.0,10)
        cell2 = gridandcell.Cell(0, 1, 5, 15, 10)
        cell3 = gridandcell.Cell(1, 0, 15, 5.0, 10)
        cell4 = gridandcell.Cell(1, 1, 15, 15, 10)
        newGrid = gridandcell.Grid2DCartesian(400, 21, 10)
        print(newGrid.get_cell(0,0))
        self.assertEqual(cell1, newGrid.get_cell(0, 0))
        self.assertEqual(cell2, newGrid.get_cell(0, 1))
        self.assertEqual(cell3, newGrid.get_cell(1, 0))
        self.assertEqual(cell4, newGrid.get_cell(1, 1))
        self.assertEqual(cell1, newGrid.whichCellAmIIn(4,3))
        self.assertEqual(cell1, newGrid.whichCellAmIIn(7, 9))
        self.assertEqual(cell3, newGrid.whichCellAmIIn(17, 7))
        self.assertEqual(cell3, newGrid.whichCellAmIIn(13, 1))
        self.assertEqual(cell2, newGrid.whichCellAmIIn(3, 13))
        self.assertEqual(cell2, newGrid.whichCellAmIIn(9, 19))
        self.assertEqual(cell4, newGrid.whichCellAmIIn(14, 13))
        self.assertEqual(cell4, newGrid.whichCellAmIIn(19, 17))
        self.assertEqual(newGrid.whichCellAmIIn(4,15), newGrid.get_cell_from_index(0,1))
        print(newGrid.whichCellAmIIn(4,15))


if __name__ == '__main__':
    unittest.main()
