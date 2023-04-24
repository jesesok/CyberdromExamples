import time
import typing
import math
import dataclasses
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


@dataclasses.dataclass
class Point:
    """
    Класс для описания точек
    """
    x: float
    y: float


class Node:
    F_VALUE = 1

    def __init__(self, center_point: Point):
        """
        Класс для описания узлов графа
        @param center_point: Координаты центра узла
        """
        self.center_point: Point = center_point
        self.neighboursDir: typing.List[Node] = []
        self.neighboursDiag: typing.List[Node] = []

        self.isDeadEnd: bool = False
        self.link_to_last_node: (Node, None) = None

        self.f_value = 0
        self.f = 0
        self.path = 0
        self.mass = 0

    def add_neighbourDir(self, neighbour):
        neighbour: Node = neighbour
        if neighbour in self.neighboursDir:
            return

        self.neighboursDir.append(neighbour)

        if self in neighbour.neighboursDir:
            return
        neighbour.neighboursDir.append(self)

    def add_neighbourDiag(self, neighbour):
        neighbour: Node = neighbour
        if neighbour in self.neighboursDiag:
            return

        self.neighboursDiag.append(neighbour)

        if self in neighbour.neighboursDiag:
            return
        neighbour.neighboursDiag.append(self)

    def print_data(self):
        print(self.center_point.x, self.center_point.y, self.f, self.path, self.mass)


class Map:
    def __init__(self):
        self.nods: typing.List[typing.List[Node]] = []

        self.open_nodes: typing.List[Node] = []  # Вершины, в которых уже были
        self.explored_nodes: typing.List[Node] = []  # Исследованные вершины

        self.__sizeNode = 1

    def create_map(self, num_node, height, weight):
        """
        Создание карты из элементов Node
        @param num_node: кол-во узлов, на которе разбивается сторона
        @param height: ширина карты в метрах
        @param weight: высота карты в метрах
        @return:
        """
        height_one_node = height / num_node
        weight_one_node = weight / num_node

        centre_x = weight_one_node / 2
        centre_y = height_one_node / 2

        for i in range(num_node):
            nods = []
            for j in range(num_node):
                x = centre_x + i * weight_one_node - 5.5
                y = centre_y + j * height_one_node - 5.5
                centre_node = Point(x=x, y=y)
                nods.append(Node(centre_node))
            self.nods.append(nods)

        for i in range(1, num_node - 1):
            for j in range(1, num_node - 1):
                self.nods[i][j].add_neighbourDir(self.nods[i + 1][j])
                self.nods[i][j].add_neighbourDir(self.nods[i - 1][j])

                self.nods[i][j].add_neighbourDir(self.nods[i][j + 1])
                self.nods[i][j].add_neighbourDir(self.nods[i][j - 1])

                self.nods[i][j].add_neighbourDiag(self.nods[i - 1][j - 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i - 1][j + 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i + 1][j - 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i + 1][j + 1])

    def add_block(self, point: Point):
        """
        Добавление на карту препятствий. Узел, которому принадлежит точка point будет помечена, как непроходимая
        @param point: Центр препятствия
        @return:
        """
        block_node = self.__point_to_node(point)
        block_node.isDeadEnd = True

    def __point_to_node(self, point: Point):
        for i in range(len(self.nods)):
            for j in range(len(self.nods[i])):
                res = self.__check_distance_in_square(point, self.nods[i][j].center_point, 0.5)
                if res:
                    return self.nods[i][j]

    def __calculation_heuristic(self, point1: Point, point2: Point):
        return math.dist((point1.x, point1.y), (point2.x, point2.y)) * 2

    def get_trajectory(self, start_point: Point, end_point: Point):
        """ Полчение траектории """
        start_node = self.__point_to_node(start_point)
        end_node = self.__point_to_node(end_point)

        if end_node.isDeadEnd:
            print('Конечная точка недостигаема')
            return

        is_end_node = False

        cur_node = start_node
        while True:
            # print("_____________________________________________________________")
            # print("cur_node ", (cur_node.center_point.x, cur_node.center_point.y))

            neighboursCurNode = cur_node.neighboursDir + cur_node.neighboursDiag
            for node in neighboursCurNode:
                F = 1 if (node in cur_node.neighboursDir) else 1.5

                if node.isDeadEnd:
                    continue

                if node in self.explored_nodes:
                    continue

                if node not in self.open_nodes:
                    self.open_nodes.append(node)

                if (node.f == 0) or (cur_node.f + F < node.f):
                    node.f += F + cur_node.f
                    node.path = self.__calculation_heuristic(end_node.center_point, node.center_point)
                    node.mass = node.f + node.path
                    node.link_to_last_node = cur_node
                else:
                    self.explored_nodes.append(node)

                # node.print_data()
                if node == end_node:
                    is_end_node = True
                    break

            if is_end_node:
                break

            next_node = self.open_nodes[0]
            for node in self.open_nodes:
                if node.mass <= next_node.mass:
                    next_node = node

            self.open_nodes.remove(next_node)

            self.explored_nodes.append(cur_node)
            cur_node = next_node

        trajectory_node = []
        node = end_node
        while True:
            trajectory_node.append(node)
            if node == start_node:
                break
            node = node.link_to_last_node

        i = 0
        while True:
            if (i >= len(trajectory_node) - 1) or (i + 1 >= len(trajectory_node)) or (i + 2 >= len(trajectory_node)):
                break
            res_check = self.__check_in_line(trajectory_node[i].center_point,
                                             trajectory_node[i + 1].center_point,
                                             trajectory_node[i + 2].center_point)
            if res_check:
                trajectory_node.remove(trajectory_node[i + 1])
                print("Точка удалена")
            else:
                i = i + 1
        new_tr = []
        for t in trajectory_node:
            new_tr.append(t.center_point)

        return new_tr

    @classmethod
    def __check_in_line(cls, point1: Point, point2: Point, point3: Point):
        if (point2.x - point1.x) * (point3.y - point1.y) - (point3.x - point1.x) * (point2.y - point1.y):

            return False
        else:
            print("Точки: ", point1, point2, point3, " лежат на одной прямой")
            return True

    @staticmethod
    def __check_distance_in_square(point1: Point, point2: Point, dist_to_collision: float) -> bool:
        """
            Проверка нахождения точки point1 внутри квадрата с центром в point2 и стороной dist_to_collision * 2
            :param point1:
            :param point2:
            :return:
        """
        if ((point2.x + dist_to_collision) > point1.x > (point2.x - dist_to_collision)) and (
                (point2.y + dist_to_collision) > point1.y > (point2.y - dist_to_collision)):
            return True
        else:
            return False


if __name__ == "__main__":

    MAP_BLOCK_LIST = [
        (-3.5, -2.5),

        (-4.5, 3.5),
        (-4.5, 3),
        (-4.5, 2.5),

        (-1.8, -0.9),
        (-1.7, 1.2),

        (-1, 3.0),
        (-1, 2.5),
        (-1.4, 3.0),
        (-1.4, 2.5),

        (-2, -3),
        (-1.5, -3),
        (-1, -3),
        (-0.5, -3),
        (-0.5, -2.5),
        (-0.5, -2),

        (4, 4.0),
        (4, 3.5),
        (4.4, 4.0),
        (4.4, 3.5),

        (2.2, 0),
        (2.2, -0.5),
        (2.2, -1),
        (2.2, -1.5),
        (2.7, 0),
        (2.7, -0.5),
        (2.7, -1),
        (2.7, -1.5),
    ]

    m = Map()
    m.create_map(22, 11, 11)

    for blockPoint in MAP_BLOCK_LIST:
        m.add_block(Point(blockPoint[0], blockPoint[1]))

    startPoint = Point(-1, -4.5)
    endPoint = Point(3, 3)

    tr = m.get_trajectory(startPoint, endPoint)
    print(tr)

    x = []
    y = []
    for t in tr:
        x.append(t.x)
        y.append(t.y)

    fig, ax = plt.subplots(figsize=(6, 6))

    ax.set_xlim([-6, 6])
    ax.set_ylim([-6, 6])
    ax.grid()

    sizeNode = 0.5
    for nodeColumn in m.nods:
        for node in nodeColumn:
            pos = node.center_point
            ax.scatter(x=pos.x, y=pos.y, marker='o', c='r', edgecolor='b')
            color = "red" if node.isDeadEnd else "white"
            ax.add_patch(Rectangle((pos.x - sizeNode / 2, pos.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                                   edgecolor='black'))

    for node in m.explored_nodes:
        pos = node.center_point
        ax.scatter(x=pos.x, y=pos.y, marker='o', c='r', edgecolor='b')
        color = "grey"
        ax.add_patch(Rectangle((pos.x - sizeNode / 2, pos.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                               edgecolor='black'))

    for i, t in enumerate(tr):
        color = "green" if i == 0 or i == (len(tr) - 1) else "blue"
        ax.scatter(x=t.x, y=t.y, marker='o', c='r', edgecolor='b')
        ax.add_patch(Rectangle((t.x - sizeNode / 2, t.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                               edgecolor='black'))

    plt.plot(x, y)
    plt.show()
