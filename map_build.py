
class Map:

    def __init__(self, map_name):
        self.map_name = map_name

    def build_map(self):
        coords = []
        objective_coord = ()
        with open(f"./maps/{self.map_name}", "r") as m_fp:
            lines = m_fp.readlines()
            for line in lines:
                match line[0]:
                    case "#":
                        # empty line so pass
                        pass
                    case '\n':
                        pass
                    case "!":
                        # '2' because of space after exclamation mark
                        objective_split = line[2:].split(", ")
                        objective_coord = (int(objective_split[0]), int(objective_split[1]))
                    case _:
                        line_split = line.strip().split(", ")

                        top_left = (int(line_split[0]), int(line_split[1]))
                        bottom_right = (int(line_split[2]), int(line_split[3]))
                        coords.append( (top_left, bottom_right) )
        return coords, objective_coord

