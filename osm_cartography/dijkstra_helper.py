#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import math, heapq


class DijkstraHelper:

    def __init__(self, root):
        self.root = None
        self.nodes, self.ways, self.relations = {}, {}, []
        self.graph = {}

        self.fill_graph()

    def calculate_distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def fill_graph(self):
        for element in self.root:
            if element.tag == 'node':
                node_id, local_x, local_y = element.attrib['id'], None, None
                for tag in element.findall('tag'):
                    if tag.attrib['k'] == 'local_x': local_x = float(tag.attrib['v'])
                    if tag.attrib['k'] == 'local_y': local_y = float(tag.attrib['v'])
                if local_x and local_y: self.nodes[int(node_id)] = (local_x, local_y)
            elif element.tag == 'way':
                self.ways[int(element.attrib['id'])] = [int(nd.attrib['ref']) for nd in element.findall('nd')]
            elif element.tag == 'relation':
                self.relations.append([(int(m.attrib['ref']), m.attrib['role']) for m in element.findall('member')])

        for way in self.ways.values():
            for i in range(len(way) - 1):
                node1, node2 = way[i], way[i + 1]
                if node1 in self.nodes and node2 in self.nodes:
                    dist = self.calculate_distance(self.nodes[node1], self.nodes[node2])
                    self.graph.setdefault(node1, []).append((node2, dist))
                    self.graph.setdefault(node2, []).append((node1, dist))

    def get_point_id(self, point_coordinates):
        # search for nearest node (it might have be aproxmately)
        min_distance = math.inf
        nearest_node = None

        for node_id, node_coordinates in self.nodes.items():
            dist = self.calculate_distance(point_coordinates, node_coordinates)
            if dist < min_distance:
                min_distance = dist
                nearest_node = node_id
        return nearest_node

    def dijkstra(self, start, end):
        # Инициализация расстояний и предшественников
        distances = {node: math.inf for node in self.graph}
        distances[start] = 0
        predecessors = {node: None for node in self.graph}

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
            for neighbor, weight in self.graph[current_node]:
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
