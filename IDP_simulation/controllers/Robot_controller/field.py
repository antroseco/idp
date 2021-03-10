import numpy as np


class Field:

    def __init__(self, colour='red'):

        self.colour = colour
        self.sizex = 0.4
        self.sizey = 0.4
        self.x = 0
        self.y = 0.4
        self.box_positions = []

        if colour == 'green':
            self.y = -0.4
            self.box_positions.append((self.x + 0.05, self.y))
            self.box_positions.append((self.x + 0.05, self.y + 0.1))
            self.box_positions.append((self.x - 0.05, self.y))
            self.box_positions.append((self.x - 0.05, self.y + 0.1))
        else:  # box red
            self.box_positions.append((self.x + 0.05, self.y))
            self.box_positions.append((self.x + 0.05, self.y - 0.1))
            self.box_positions.append((self.x - 0.05, self.y))
            self.box_positions.append((self.x - 0.05, self.y - 0.1))

        self.box_positions = np.array(self.box_positions)

        return

    def available(self):
        if self.box_positions.size == 0:
            return False
        return True

    def closest_box_position(self, coord):
        """
        returns closest available box position
        input are 3D coordinates of the robot
        """

        min_dist = float('inf')
        min_index = 0

        for i in range(self.box_positions.shape[0]):
            dist = (coord[0] - self.box_positions[i][0])**2 + (coord[1] - self.box_positions[i][1])**2
            if dist < min_dist:
                min_dist = dist
                min_index = i

        pos = self.box_positions[min_index]

        # remove this place from array, it becomes unavailable
        self.box_positions = np.delete(self.box_positions, min_index, axis=0)

        return pos

    def get_to_field(self, coord):
        """
        gives a few coordinates that should be followed to get to the specific location
        in the field without moving the boxes that are already in there
        """

        final_pos = self.closest_box_position(coord)

        # if x value is positive
        if final_pos[0] > 0:
            intermediate_pos = (self.x + self.sizex / 2 + 0.2, final_pos[1])
        else:
            intermediate_pos = (self.x - self.sizex / 2 - 0.2, final_pos[1])

        return intermediate_pos, final_pos
