from polygonal_roadmaps import geometry
from shapely.geometry import Point, LineString

def calculate_occupied_space(map_file, inflation=0.15, simplification=0.05):
    _, o = geometry.read_obstacles(map_file)
    o = o.buffer(inflation)
    o = o.simplify(simplification)
    return o
    
def path_2d(path):
    return LineString(path)


if __name__ == "__main__":
    pass