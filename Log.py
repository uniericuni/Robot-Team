class Log:

    def __init__(self, log_on):
        self.log_on = log_on

    def msg(self, msg_type, msg):
        if self.log_on:
            print '[' + msg_type + '] ' + msg
