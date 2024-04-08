from Node import NodeType


class Local_DASP:
    def __init__(self, model, tours, d_station):
        self.model = model
        self.tours = tours
        self.d_station = d_station
        # print(d_station)
        self.V_start = self.get_starter_nodes()
        self.Vn = self.get_ds_customers()
        self.V_end = self.get_end_nodes()
        # self.Vos, self.Voe = self.get_outlier_nodes()

    def get_starter_nodes(self):
        starter_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            # for each node
            for j in range(1, len(self.tours[i])):
                node = self.tours[i][j]
                # print(f"Station= {d_station} location = {self.v[d_station].location}")
                # print(f"Node {node.index} Location = {node.location}")
                # print("distance=", node.node_distance(self.v[d_station]))
                # print("eps=", self.eps / 2)
                # print("RESULT= ", node.node_distance(self.v[d_station]) <= self.eps / 2)
                if (node.node_type == NodeType.CUSTOMER
                        and node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2):
                    starter_nodes.append(self.tours[i][j - 1])
                    break
        # print(starter_nodes)
        return starter_nodes

    def get_ds_customers(self):
        Vn = []
        for tour in self.tours:
            for node in tour:
                if node.node_type == NodeType.CUSTOMER and node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2:
                    Vn.append(node)
        return Vn

    def get_end_nodes(self):
        end_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            # for each node
            for j in range(len(self.tours[i]) - 2, -1, -1):
                node = self.tours[i][j]
                if (node.node_type == NodeType.CUSTOMER and
                        node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2):
                    end_nodes.append(self.tours[i][j + 1])
                    break
        # print("end nodes: ", end_nodes)
        return end_nodes

    def solve(self):
        print("start", self.V_start)
        print("end", self.V_end)
        print("tour_nodes", self.Vn)
