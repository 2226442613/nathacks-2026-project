import argparse
from .cli import add_dashboard_command,run_dashboard
parser=argparse.ArgumentParser(description='Hospital Guardian local dashboard')
commands=parser.add_subparsers(dest='command',required=True)
add_dashboard_command(commands)
try:run_dashboard(parser.parse_args())
except KeyboardInterrupt:pass
