from typing import List, Tuple
import math
# from bisect import bisect_left
from helpers.binary_search import bisect_left

from helpers.trip import Walk, Bus


class ATF:
    __slots__ = "walk", 'buses', 'size'

    def __init__(self, walk: Walk = None, buses: List[Bus] = None):
        """
        Arrival Time Function.
        More detailed you can find
         by link: https://oliviermarty.net/docs/olivier_marty_contraction_hierarchies_rapport.pdf
        :param walk: Walk profile between nodes
        :param buses: List[Bus] , List of Buses between 2 nodes
        """

        self.walk = walk
        self.buses = buses
        self.size = len(buses)

    def cut(self):
        """
        Method for filtering dominated connections in ATF
        More detailed this procedure described by the link
         by link: https://oliviermarty.net/docs/olivier_marty_contraction_hierarchies_rapport.pdf
        :return:
        """
        r = list()
        i = 0
        while i < self.size:
            if r:
                d, a = r[-1].d, r[-1].a
                if self.buses[i].a > a:
                    if d < self.buses[i].d:
                        if self.walk:
                            if self.buses[i].a - self.buses[i].d <= self.walk.w:
                                r.append(self.buses[i])
                        else:
                            r.append(self.buses[i])
                    i += 1
                else:
                    r.pop()
            else:
                if self.walk:
                    if self.buses[i].a - self.buses[i].d <= self.walk.w:
                        r += [self.buses[i]]
                else:
                    r += [self.buses[i]]
                i += 1
        self.buses = r
        self.size = len(r)

    def composition(self, f):
        """
        Composition of 2 ATF's functions
        :param f: ATF function
        :return: self(f)
        """
        cc, cw, wc = [], [], []

        walk = None
        if self.walk and f.walk:
            w = self.walk.w + f.walk.w
            w_nodes = f.walk.nodes + self.walk.nodes[1:]
            walk = Walk(nodes=w_nodes, w=w)

        i = 0
        j = 0
        while i < f.size and j < self.size:
            if i + 1 < f.size and f.buses[i + 1].a <= self.buses[j].d:
                if self.walk:
                    new_a = f.buses[i].a + self.walk.w
                    if new_a < self.buses[j].a:
                        cw_c = (f.buses[i].d, new_a)
                        cw_nodes = f.buses[i].nodes + self.walk.nodes[1:]
                        cw_route_names = f.buses[i].route_names + self.walk.route_names
                        bus = Bus(nodes=cw_nodes, c=cw_c, route_names=cw_route_names)
                        cw.append(bus)

                i += 1
            elif f.buses[i].a <= self.buses[j].d:
                big_path = True

                if self.walk:
                    new_a = f.buses[i].a + self.walk.w
                    if new_a < self.buses[j].a:
                        cw_c = (f.buses[i].d, new_a)
                        cw_nodes = f.buses[i].nodes + self.walk.nodes[1:]
                        cw_route_names = f.buses[i].route_names + self.walk.route_names
                        bus = Bus(nodes=cw_nodes, c=cw_c, route_names=cw_route_names)
                        cw.append(bus)
                        big_path = False

                if f.walk:
                    new_d = self.buses[j].d - f.walk.w
                    wc_c = (new_d, self.buses[j].a)
                    wc_nodes = f.walk.nodes + self.buses[j].nodes[1:]
                    wc_route_names = f.walk.route_names + self.buses[j].route_names
                    bus = Bus(nodes=wc_nodes, c=wc_c, route_names=wc_route_names)
                    wc.append(bus)

                if big_path:
                    cc_c = (f.buses[i].d, self.buses[j].a)
                    cc_nodes = f.buses[i].nodes + self.buses[j].nodes[1:]
                    cc_route_names = f.buses[i].route_names + self.buses[j].route_names
                    bus = Bus(nodes=cc_nodes, c=cc_c, route_names=cc_route_names)
                    cc.append(bus)
                i += 1
                j += 1
            else:
                if f.walk:
                    wc_c = (self.buses[j].d - f.walk.w, self.buses[j].a)
                    wc_nodes = f.walk.nodes + self.buses[j].nodes[1:]
                    wc_route_names = f.walk.route_names + self.buses[j].route_names
                    bus = Bus(nodes=wc_nodes, c=wc_c, route_names=wc_route_names)
                    wc.append(bus)
                j += 1

        for s in range(i, f.size):
            if self.walk:
                cw_c = (f.buses[s].d, f.buses[s].a + self.walk.w)
                cw_nodes = f.buses[s].nodes + self.walk.nodes[1:]
                cw_route_names = f.buses[s].route_names + self.walk.route_names
                bus = Bus(nodes=cw_nodes, c=cw_c, route_names=cw_route_names)
                cw.append(bus)

        for s in range(j, self.size):
            if f.walk:
                wc_c = (self.buses[s].d - f.walk.w, self.buses[s].a)
                wc_nodes = f.walk.nodes + self.buses[s].nodes[1:]
                wc_route_names = f.walk.route_names + self.buses[s].route_names
                bus = Bus(nodes=wc_nodes, c=wc_c, route_names=wc_route_names)
                wc.append(bus)

        c = cc + cw + wc
        c.sort()
        if walk or c:
            g = ATF(walk=walk, buses=c)
            g.cut()
            return g

    def arrival(self, t: int) -> Tuple[int, List[int], List[str]]:
        """
        Calculate arrival time to next station
        :param t: start_time
        :return:
        """
        l = math.inf
        sequence_nodes = []
        route_names = []
        start_index = bisect_left(self.buses, t, key=lambda x: x.d, lo=0, hi=self.size)
        if start_index < self.size:
            l = self.buses[start_index].a
            sequence_nodes = self.buses[start_index].nodes
            route_names = self.buses[start_index].route_names
        if self.walk:
            walk_time = t + self.walk.w
            if walk_time < l:
                return walk_time, self.walk.nodes, self.walk.route_names
        return l, sequence_nodes, route_names


def min_atf(f1: ATF, f2: ATF) -> ATF:
    """
    Minimum of 2 ATF function.
    Description could be found by link :
    https://oliviermarty.net/docs/olivier_marty_contraction_hierarchies_rapport.pdf
    :param f1:
    :param f2:
    :return:
    """
    if f1.walk and f2.walk:
        if f1.walk.w > f2.walk.w:
            walk = f2.walk
        else:
            walk = f1.walk
    elif f1.walk:
        walk = f1.walk
    else:
        walk = f2.walk

    buses = f1.buses + f2.buses
    buses.sort()
    g = ATF(walk=walk, buses=buses)
    g.cut()
    return g
