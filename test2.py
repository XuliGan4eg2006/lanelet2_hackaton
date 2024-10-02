import xml.etree.ElementTree as ET
import math, heapq
import itertools

# Парсинг XML
tree = ET.parse('maps/map.osm')
root = tree.getroot()

nodes, ways, relations = {}, {}, []
only_nodes_ids = []

for element in root:
    if element.tag == 'node':
        node_id, local_x, local_y = element.attrib['id'], None, None
        for tag in element.findall('tag'):
            if tag.attrib['k'] == 'local_x': local_x = float(tag.attrib['v'])
            if tag.attrib['k'] == 'local_y': local_y = float(tag.attrib['v'])
        if local_x and local_y: nodes[int(node_id)] = (local_x, local_y)
        only_nodes_ids.append(int(node_id))
    elif element.tag == 'way':
        ways[int(element.attrib['id'])] = [int(nd.attrib['ref']) for nd in element.findall('nd')]
    elif element.tag == 'relation':
        relations.append([(int(m.attrib['ref']), m.attrib['role']) for m in element.findall('member')])


# print(ways)
# print(relations)


# Построение графа
def calculate_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)


lines = {}

#adding lines like (node1, node2) = weight
for way in ways.values():
    damn = list(itertools.combinations(way, 2))
    lines.update({(node1, node2): calculate_distance(nodes[node1], nodes[node2]) for node1, node2 in damn})

print(lines)


def minimum(dict):
    min_key = list(dict.keys())[0]
    for i in list(dict.keys())[1:]:
        if dict[i] < dict[min_key]:
            min_key = i
    return (min_key)


def dijkstra(airports, lines, start, end):
    unexplored = {airport: float('inf') for airport in airports}
    unexplored[start] = 0
    while len(unexplored) != 0:
        explore = minimum(unexplored)
        if explore == end:
            break
        else:
            for path in lines.items():
                if path[0][0] == explore:
                    if path[0][1] in unexplored.keys():
                        check_time = unexplored[path[0][0]] + path[1]
                        if check_time < unexplored[path[0][1]]:
                            unexplored[path[0][1]] = check_time
                elif path[0][1] == explore:
                    if path[0][0] in unexplored.keys():
                        check_time = unexplored[path[0][1]] + path[1]
                        if check_time < unexplored[path[0][0]]:
                            unexplored[path[0][0]] = check_time
            del unexplored[explore]

    return (unexplored[explore])

print(dijkstra(only_nodes_ids, lines, 1, 144))