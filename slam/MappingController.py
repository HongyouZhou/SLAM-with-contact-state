import networkx as nx


class MappingController:
    # this mapping class should have the following functions:
    #  1. first generate transition matrix and probability matrix from config for the prediction
    #  2. Each contact state should have corresponded sensory patterns for the update
    #  3. It should have a database to save the sensory patterns for each contact state
    #  4. Generate action based on the belief over contact states
    def __init__(self, config):
        self.config = config
        self.map = nx.Graph()
        self.all_css = []
        self.num_cs = len(self.all_css)
        # get all states which don't successors
        self.all_terminal_states = []
        self.cs_map = config["cs_map"]
        self.terminate_cs = config["terminate_state"]
        self.trans_prob_mat = None
        self.initial_transition_matrix()
        self.measurement_model = config["cs_measurement_model"]

    def check_measurement(self):
        """ compare the observation with database, to decide whether add a node into the map
        """

    def add_nodes(self, parent_cs, node_name):
        """ compare the observation with database, to decide whether add a node into the map
        """
        pass

    def initial_transition_matrix(self):
        """ create three transition matrices include transition probability, orientation and force patterns
        """
        # get all contact states
        pass
