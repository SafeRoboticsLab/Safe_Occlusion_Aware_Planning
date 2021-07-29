from mapping import OccupancyMap, RoadNetwork


filename = "data/map/Town01_overtake.ot"

occupancy_map = OccupancyMap(filename=filename)
occupancy_map.octree

ego  = [380.73,  -2,  2]
pt =  [392.37,  -80,   2]

print(occupancy_map.check_visibility(ego, pt, True))
# for it in occupancy_map.octree.begin_leafs():
#     print(occupancy_map.octree.isNodeOccupied(it))
#     print(it.getCoordinate())