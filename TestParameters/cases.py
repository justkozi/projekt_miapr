from math import pi


class TestParameters:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):
        #x, y, angle
        self.start_pos = [7.5, 2.4, -pi/2]
        self.end_pos = [7, 8, pi]
        # x, y, w, h
        # self.obs = [
        #     [2, 3, 6, 0.1]
        #     # [2, 3, 0.1, 1.5],
        #     # [4.3, 0, 0.1, 1.8],
        #     # [6.5, 1.5, 0.1, 1.5],
        #     # [0, 6, 3.5, 0.1],
        #     # [5, 6, 5, 0.1]
        # ]

        self.obs = [
            [2, 9, 2, 0.1],
            [2, 5, 0.1, 4],
            [4, 7, 0.1, 2],
            [2, 7, 2, 0.1],

            [6, 6, 2, 0.1],
            [6, 2, 0.1, 4],
            [8, 4, 0.1, 2],
            [6, 4, 2, 0.1]
        ]
