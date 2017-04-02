class Log:

    def __init__(self, display):
        self.display = display
        self.history = []

    def msg(self, msg_type, msg):
        msg = '[' + msg_type + '] ' + msg
        self.history.append(msg)
        if self.display:
            print msg
    
    def history(self):
        for msg in self.history:
            print msg

    def clear(self):
        self.history = []
