import argparse
import os


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Replay a phase 5 bag without starting Gazebo.',
    )
    parser.add_argument('bag_dir')
    parser.add_argument(
        '--clock',
        action='store_true',
        help='Publish /clock from recorded bag timestamps while replaying.',
    )
    parsed = parser.parse_args(args)

    command = ['ros2', 'bag', 'play', parsed.bag_dir]
    if parsed.clock:
        command.append('--clock')
    os.execvp(command[0], command)


__all__ = ['main']
