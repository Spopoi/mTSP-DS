from Node import NodeType


class Local_DASP:
    def __init__(self, model, tours, d_station):
        self.model = model
        self.tours = tours
        self.d_station = d_station
        # print(d_station)
        # TODO: check local_tours, may not be present ds in the local tour
        self.local_tours = []
        self.V_OS = []
        self.V_OE = []
        self.O = []
        self.V_start = self.get_starter_nodes()
        self.Vn = self.get_ds_customers()
        self.V_end = self.get_end_nodes()
        self.get_outlier_nodes()

        self.Vl = []
        self.Vl.extend(self.V_start)
        self.Vl.extend(self.Vn)
        self.Vl.extend(self.model.v[d_station])
        self.Vl.extend(self.V_OE)

        self.Vr = []
        self.Vr.extend(self.Vn)
        self.Vr.extend(self.model.v[d_station])
        self.Vr.extend(self.V_OS)
        self.Vr.extend(self.V_end)

        # self.Vos, self.Voe = self.get_outlier_nodes()

    def get_starter_nodes(self):
        starter_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            self.local_tours.append([])
            # for each node
            for j in range(1, len(self.tours[i])):
                node = self.tours[i][j]
                # print(f"Station= {d_station} location = {self.v[d_station].location}")
                # print(f"Node {node.index} Location = {node.location}")
                # print("distance=", node.node_distance(self.v[d_station]))
                # print("eps=", self.eps / 2)
                # print("RESULT= ", node.node_distance(self.v[d_station]) <= self.eps / 2)
                if node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2:
                    starter_nodes.append((i, self.tours[i][j - 1]))
                    self.local_tours[i] = self.tours[i][j:]
                    break
        return starter_nodes

    def get_ds_customers(self):
        Vn = []
        for tour in self.tours:
            for node in tour:
                if node.node_type == NodeType.CUSTOMER and node.node_distance(
                        self.model.v[self.d_station]) <= self.model.eps / 2:
                    Vn.append(node)
        return Vn

    # TODO: rimettere i depot nei local_tours
    def get_end_nodes(self):
        end_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            # for each node
            for j in range(len(self.tours[i]) - 2, -1, -1):
                node = self.tours[i][j]
                if node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2:
                    end_nodes.append((i, self.tours[i][j + 1]))

                    # Rimuovi tutti gli elementi dopo 'node' in 'self.local_tours[i]'
                    node_index = self.local_tours[i].index(node)
                    self.local_tours[i] = self.local_tours[i][:node_index + 1]
                    break
        # print("end nodes: ", end_nodes)
        return end_nodes

    def solve(self):
        print("start", self.V_start)
        # print("end", self.V_end)
        # print("tour_nodes", self.Vn)
        print("Local tours= ", self.local_tours)
        print("Outliers= ", self.O)


    # def get_outlier_nodes(self):
    #     for i in range(len(self.local_tours)):
    #         # for each node
    #         for j in range(len(self.local_tours[i])):
    #             node = self.local_tours[i][j]
    #             if node.node_type == NodeType.CUSTOMER and node.node_distance(self.model.v[self.d_station]) > self.model.eps / 2:
    #                 self.V_OS.append(node)
    #                 if len(self.local_tours[i]) == 1:
    #                     oe_node = node
    #                 else:
    #                     oe_node = self.get_end_outlier(i, j+1)
    #                 self.V_OE.append(oe_node)
    #                 # TODO: verify it works
    #                 j = self.local_tours[i].index(oe_node)+1
    #                 self.O.append((node, oe_node))
    def get_outlier_nodes(self):
        for i in range(len(self.local_tours)):
            j = 0
            while j < len(self.local_tours[i]):
                node = self.local_tours[i][j]
                if node.node_type == NodeType.CUSTOMER and node.node_distance(
                        self.model.v[self.d_station]) > self.model.eps / 2:
                    self.V_OS.append(node)
                    if len(self.local_tours[i]) == 1:
                        oe_node = node
                    else:
                        oe_node = self.get_end_outlier(i, j + 1)
                    self.V_OE.append(oe_node)
                    # TODO: verify it works
                    j = self.local_tours[i].index(oe_node) + 1
                    self.O.append((node, oe_node))
                else:
                    j += 1  # Se non è un nodo outlier, passa al prossimo

    def get_end_outlier(self, tour_k, node_index):
        # for each node
        for j in range(node_index, len(self.local_tours[tour_k])):
            node = self.local_tours[tour_k][j]
            if node.node_distance(self.model.v[self.d_station]) <= self.model.eps / 2:
                return self.local_tours[tour_k][j - 1]
