import xml.etree.ElementTree as ET
import math, heapq

# Парсинг XML
tree = ET.parse('maps/map.osm')
root = tree.getroot()

nodes, ways, relations = {}, {}, []

for element in root:
    if element.tag == 'node':
        node_id, local_x, local_y = element.attrib['id'], None, None
        for tag in element.findall('tag'):
            if tag.attrib['k'] == 'local_x': local_x = float(tag.attrib['v'])
            if tag.attrib['k'] == 'local_y': local_y = float(tag.attrib['v'])
        if local_x and local_y: nodes[int(node_id)] = (local_x, local_y)
    elif element.tag == 'way':
        ways[int(element.attrib['id'])] = [int(nd.attrib['ref']) for nd in element.findall('nd')]
    elif element.tag == 'relation':
        relations.append([(int(m.attrib['ref']), m.attrib['role']) for m in element.findall('member')])


# print(ways)
# print(relations)


# Построение графа
def calculate_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)


graph = {}

for way in ways.values():
    for i in range(len(way) - 1):
        node1, node2 = way[i], way[i + 1]
        if node1 in nodes and node2 in nodes:
            dist = calculate_distance(nodes[node1], nodes[node2])
            graph.setdefault(node1, []).append((node2, dist))
            graph.setdefault(node2, []).append((node1, dist))

print(graph)
counter = 0


# Учет направлений
# for members in relations:
#     for ref, role in members:
#         if role == 'left':
#             graph.pop(ref, None)
#         elif role == 'right':
#             graph.pop(ref, None)
#         print(counter,ref, role)
#         counter += 1


def dijkstra(graph, start, end):
    # Инициализация расстояний и предшественников
    distances = {node: math.inf for node in graph}
    distances[start] = 0
    predecessors = {node: None for node in graph}

    # Очередь с приоритетом для выбора следующей вершины
    # (расстояние, вершина)
    pq = [(0, start)]

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        # Если достигли конечной точки, можно завершать
        if current_node == end:
            break

        # Если уже нашли более короткий путь, пропускаем
        if current_distance > distances[current_node]:
            continue

        # Проверяем всех соседей текущей вершины
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight

            # Если нашли более короткий путь, обновляем
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    # Восстановление пути
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = predecessors[current]
    path.reverse()

    # Если путь не найден, возвращаем пустой список
    if not path or path[0] != start:
        return math.inf, []

    return distances[end], path


print(dijkstra(graph, 1, 144))


