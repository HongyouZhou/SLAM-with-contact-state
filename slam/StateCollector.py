import os
import sys
file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)

import pandas as pd
import numpy as np


class StateCollector:

    def __init__(self,
                 state_collector_name,
                 experiment_dir):

        # The controller governing the robot execution
        self.state_collector_name = state_collector_name
        # The experimental setup determining the joints being used
        self.experiment_dir = experiment_dir
        # Determine the data structures to hold the collected state and send actuation command
        self.df_state = pd.DataFrame()
        # Create dirs to save data and plots
        self.state_collector_dir, self.state_collector_plot_dir = self.get_state_collector_dirs()
        # Data name is used to name the output data and plots
        self.data_name = None
        # Initialization of the data visualizer
        self.plot_features = ['d_ee_v_ee', 'ee_v_ee', 'base_p_ee', 'base_v_ee', 'ft_robot', 'ft_human']
        # Clear state df
        self.reset()

    def get_state_collector_dirs(self):
        """
        create dirs to save data and plots
        """
        state_collector_dir = os.path.join(self.experiment_dir+"/state_collector/",
                                           self.state_collector_name + "_sc")
        state_collector_plot_dir = os.path.join(state_collector_dir, "plot")
        if not os.path.exists(state_collector_dir):
            os.makedirs(state_collector_dir)
        if not os.path.exists(state_collector_plot_dir):
            os.makedirs(state_collector_plot_dir)
        return state_collector_dir, state_collector_plot_dir

    def add_data(self, data_frame):
        # Get most recent readings from queue and add them to trajectory
        self.df_state = self.df_state.append(data_frame, ignore_index=True)

    def export_as_pkl(self, output_name, overwrite=False):
        index = 0
        export_name = os.path.join(self.state_collector_dir, output_name + "_{}.pkl".format(index))
        # incrementally change name
        if not overwrite:
            while os.path.isfile(export_name):
                index += 1
                export_name = os.path.join(self.state_collector_dir, output_name + "_{}.pkl".format(index))
            # define data_name to create corresponding plots
            self.data_name = output_name + "_{}".format(index)

        self.df_state.to_pickle(export_name)
        print("Output pickle file: ", export_name)

    def split_action_state(self, action_name=None):
        """ split df into action data and observed state data after injection of action
        """
        self.action_data = self.df_state[self.df_state[action_name] != None]
        self.state_data = self.df_state[self.df_state[action_name] == None]

    def vstack_df(self, df):
        """ stack data vertical for plotting
        """
        data_dict = dict()
        for feature in self.plot_features:
            data_dict[feature] = np.vstack([df[feature].to_list()])
        return data_dict

    def plot_df_state(self, plot_name, df=None):
        """ if plot_name is none, then the self.data_name is used. This is defined in self.export_as_pkl
        """

        if self.data_name is not None:
            plot_name = self.data_name + "_" + plot_name
        if df is None:
            df = self.df_state

        data_dict = self.vstack_df(df)

    def reset(self):
        self.df_state = pd.DataFrame()

    def get_state_action_traj(self):
        return self.df_state
