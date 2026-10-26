"""Command-line utility for reproducibly training the bundled model."""

import argparse
import os

from ai_delivery_robot.color_model import train_color_classifier


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Train the package color Random Forest model.'
    )
    parser.add_argument(
        '--output', required=True,
        help='Output path for the OpenCV model YAML file.',
    )
    parsed = parser.parse_args(args)
    output = os.path.abspath(parsed.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    train_color_classifier().save(output)
    print(f'saved trained color classifier to {output}')


if __name__ == '__main__':
    main()
