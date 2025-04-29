
class Map:

    def __init__(self, map_name):
        self.map_name = map_name

    def build_map(self):
        print("reads in file")
        coords = []
        with open(f"./maps/{self.map_name}", "r") as m_fp:
            lines = m_fp.readlines()
            for line in lines:
                match line[0]:
                    case "#":
                        # empty line so pass
                        pass
                    case _:
                        line_split = line.strip().split(", ")
                        # print(line_split)
                        top_left = (int(line_split[0][1:]), int(line_split[1]))
                        bottom_right = (int(line_split[2]), int(line_split[3][:-1]))
                        coords.append( (top_left, bottom_right) )
                # print("while")
        return coords

