from networkx.classes.digraph import DiGraph
from networkx.algorithms.shortest_paths import dijkstra_path, dijkstra_path_length
from math import sqrt, ceil

def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001


def reach_waypoint(pose, waypoint, margin):
    if distance(pose, waypoint) <= margin:
        return True
    else:
        return False


class MotionFts(DiGraph):
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, label) in node_dict.iteritems():
            self.add_node(n, label=label, status='confirmed')

    def add_un_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in self.nodes():
            # self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)

    def dijkstra_path(self, source, reg_label):
        targets = [node for node in self.nodes() if reg_label in self.nodes[node]['label']]
        opt = targets[0]
        for target in targets[1:]:
            if dijkstra_path_length(self, source, target) < dijkstra_path_length(self, source, opt):
                opt = target
        return dijkstra_path(self, source, opt)

    def add_di_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
        for node in self.nodes():
            # self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)

    def add_un_edges_by_ap(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_ap = edge[0]
            t_ap = edge[1]
            f_nodes = [n for n in self.nodes() if f_ap in self.nodes[n]['label']]
            if len(f_nodes) > 1:
                print 'ambiguity more than one with f_ap %s, see %s' % (f_ap, str(f_nodes))
            else:
                f_node = f_nodes[0]
            t_nodes = [n for n in self.nodes() if t_ap in self.nodes[n]['label']]
            if len(t_nodes) > 1:
                print 'ambiguity more than one with t_ap %s, see %s' % (t_ap, str(t_nodes))
            else:
                t_node = t_nodes[0]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in self.nodes():
            # self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)

    def add_full_edges(self, unit_cost=1):
        for f_node in self.nodes():
            for t_node in self.nodes():
                dist = ceil(distance(f_node, t_node))
                if (f_node, t_node) not in self.edges():
                    self.add_edge(f_node, t_node, weight=dist*unit_cost)

    def set_initial(self, pose):
        init_node = self.closest_node(pose)
        self.graph['initial'] = {init_node}
        return init_node

    def closest_node(self, pose):
        node = min(self.nodes(), key=lambda n: distance(n, pose))
        return node

    def update_after_region_change(self, sense_info, com_info, margin=10):
        # sense_info = {'label':set((x,y), l', l'_)), 'edge':(set(add_edges), set(del_edges))}
        # com_info = set((x,y), l', l'_))
        # margin for adding new nodes, NOTE units!
        changed_regs = set()
        # label udpate
        label_info = sense_info['label']
        label_info.update(com_info)
        for mes in label_info:
            if mes[1]:
                close_node = self.closest_node(mes[0])
                if distance(close_node, mes[0])>margin:
                    self.add_node(mes[0], mes[1])
                else:
                    old_label = self.nodes[close_node]['label']
                    new_label = old_label.union(mes[1]).difference(mes[2])
                    if old_label != new_label:
                        self.nodes[close_node]['label'] = set(new_label)
                        self.nodes[close_node]['status'] = 'notconfirmed'
                        changed_regs.add(close_node)
        # edges udpate
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            self.add_edge(e[0], e[1], weight=distance(e[0], e[1]))
            self.nodes[close_node]['status'] = 'notconfirmed'
            changed_regs.add(e[0])
        for e in edge_info[1]:
            self.remove_edge(e[0], e[1])
            changed_regs.add(e[0])
            self.nodes[close_node]['status'] = 'notconfirmed'
        return changed_regs