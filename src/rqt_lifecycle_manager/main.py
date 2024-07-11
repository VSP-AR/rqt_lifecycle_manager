#!/usr/bin/env python3

import sys

from rqt_gui.main import Main

def main():
    sys.exit(Main().main(sys.argv, standalone='rqt_lifecycle_manager.lifecycle_manager_gui.LifecycleManagerGui'))
    
if __name__ == '__main__':
    main()