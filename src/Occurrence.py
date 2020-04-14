from colorama import Fore, Style


class Occurrence:
    """ Wrapper to associate certain string with a file and a line number """

    def __init__(self, file, line_number, content):
        self.file = file
        self.line_number = line_number
        self.content = content

    def totext(self, error=False):
        c = Style.BRIGHT + (Fore.RED if error else Fore.BLUE)
        res = c + self.content + Style.RESET_ALL + " at " + self.file
        if self.line_number:
            res += ":" + str(self.line_number)
        return res

    def __str__(self):
        return self.totext()

    def __repr__(self):
        return str(self)
