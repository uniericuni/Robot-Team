# ====================================================================================
# Log.py
#   System log messager.
# ====================================================================================
class Log:

    # Initiation. Display: bool, whether to display log or not.
    def __init__(self, display):
        self.display = display
        self.history = []

    # Print message if display is true.
    def msg(self, msg_type, msg):
        msg = '[' + msg_type + '] ' + msg
        self.history.append(msg)
        if self.display:
            print msg
    
    # Print all message in history.
    def history(self):
        for msg in self.history:
            print msg

    # Clear all the history.
    def clear(self):
        self.history = []
