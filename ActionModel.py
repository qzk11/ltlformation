from boolean_formulas.parser import parse as parse_guard


class ActionModel(object):
    # action_dict = {act_name: (cost, guard_formula, label)}
    def __init__(self, action_dict):
        self.raw = action_dict
        self.action = dict()
        for act_name, attrib in action_dict.iteritems():
            cost = attrib[0]
            guard_formula = attrib[1]
            guard_expr = parse_guard(guard_formula)
            # print guard_expr
            label = attrib[2]
            self.action[act_name] = (cost, guard_expr, label)
        self.action['None'] = (0, parse_guard('1'), set())

    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.iteritems():
            if attrib[1].check(ts_node_label):
                allow_action.add(act_name)
        return allow_action
