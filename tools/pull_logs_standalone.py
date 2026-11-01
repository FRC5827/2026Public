"""Command-line entry point for the roboRIO log puller."""

'''preset for the standalone version of the log puller that waits for connection, shows status, and waits on completion.'''

import sys

from pull_logs import main


if __name__ == "__main__":
    sys.argv.insert(1, "--wait-for-connection")
    sys.argv.insert(2, "--status")
    sys.argv.insert(3, "--wait-on-complete")
    main()
