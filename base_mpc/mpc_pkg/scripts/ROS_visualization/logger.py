import rospy
from termcolor import colored


class Logger():
    def __init__(self, prefix, log_frequence, enable_logging=True):
        self.prefix = prefix
        self.log_frequence = log_frequence
        self.enable_logging = enable_logging
        self.log_counts = 0
        para_name = self.prefix + "_enable_logging"
        self.enable_logging = rospy.get_param(para_name, True)

    def log(self, msg, type=0):
        """
            type:
              -1 -> error
               0 -> normal
               1 -> important
        """

        if self.enable_logging:
            prefix_temp = self.prefix + ": "
            if type == 0 and self.log_counts % self.log_frequence == 0:

                print(prefix_temp, msg)

            elif type == 1:

                print(colored(prefix_temp, 'blue'))
                print(colored(msg, 'blue'))

            if type == -1:
                print(colored(prefix_temp, 'red'))
                print(colored(msg, 'red'))

            if msg == "end":
                self.log_counts += 1;
