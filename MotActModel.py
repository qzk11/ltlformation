from networkx.classes.digraph import DiGraph


class MotActModel(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].node[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if (reg in self.graph['region'].graph['initial']) and (act == 'None'):
                self.graph['initial'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        reg = self.nodes[prod_node]['region']
        act = self.nodes[prod_node]['action']
        return reg, act

    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')

    def build_full(self):
        for reg in self.graph['region'].nodes():
            for act in self.graph['action'].action.iterkeys():
                prod_node = self.composition(reg, act)
                # actions
                if act == 'None':
                    label = self.graph['region'].node[reg]['label']
                    for act_to in self.graph['action'].allowed_actions(label):
                        prod_node_to = self.composition(reg, act_to)
                        if act_to != 'None':
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label=act_to, marker='visited')
                        else:
                            # this is a self-loop
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label='goto', marker='visited')
                # motions
                for reg_to in self.graph['region'].successors(reg):
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label='goto', marker='visited')
        print 'full TS model constructed with %d states and %s transitions' % (len(self.nodes()), len(self.edges()))

    def fly_successors(self, prod_node):
        reg, act = self.projection(prod_node)
        # been visited before, and hasn't changed
        if ((self.nodes[prod_node]['marker'] == 'visited') and
                (self.graph['region'].node[self.nodes[prod_node]['region']]['status'] == 'confirmed')):
            for prod_node_to in self.successors(prod_node):
                yield prod_node_to, self.edges[prod_node, prod_node_to]['weight']
        else:
            self.remove_edges_from(self.out_edges(prod_node))
            # actions
            label = self.graph['region'].node[reg]['label']
            for act_to in self.graph['action'].allowed_actions(label):
                prod_node_to = self.composition(reg, act_to)
                cost = self.graph['action'].action[act_to][0]
                self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
                yield prod_node_to, cost
            # motions
            for reg_to in self.graph['region'].successors(reg):
                if reg_to != reg:
                    prod_node_to = self.composition(reg_to, 'None')
                    cost = self.graph['region'][reg][reg_to]['weight']
                    self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')
                    yield prod_node_to, cost
            self.graph['region'].node[self.nodes[prod_node]['region']]['status'] = 'confirmed'
            self.nodes[prod_node]['marker'] = 'visited'
        # print 'full FTS model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges()))

    def fly_predecessors_iter(self, prod_node):
        reg, act = self.projection(prod_node)
        # actions
        label = self.graph['region'].node[reg]['label']
        if act in self.graph['action'].allowed_actions(label):
            for f_act in self.graph['action'].action.iterkeys():
                f_prod_node = self.composition(reg, f_act)
                cost = self.graph['action'].action[act][0]
                self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
                yield f_prod_node, cost
        # motions
        if act == 'None':
            # for f_reg in self.graph['region'].predecessors_iter(reg):
            for f_reg in self.graph['region'].predecessors(reg):
                if f_reg != reg:
                    for f_act in self.graph['action'].action.iterkeys():
                        f_prod_node = self.composition(f_reg, f_act)
                        cost = self.graph['region'][f_reg][reg]['weight']
                        self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')
                        yield f_prod_node, cost
