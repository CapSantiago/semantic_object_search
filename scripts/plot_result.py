#!/usr/bin/env python3
"""Render a saved course occupancy map and actual search result as a PNG."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result')
    parser.add_argument('--map', default='maps/course_house.yaml')
    parser.add_argument('--output', default='results/validation/semantic_map.png')
    args = parser.parse_args()
    result = json.loads(Path(args.result).read_text())
    map_file = Path(args.map)
    info = yaml.safe_load(map_file.read_text())
    if abs(info['origin'][2]) > 1e-6:
        raise ValueError('PNG preview expects the axis-aligned course map; use semantic_map.svg for rotated maps')
    image = plt.imread(map_file.parent / info['image'])
    height, width = image.shape[:2]
    x, y, _ = info['origin']
    resolution = info['resolution']
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(image, cmap='gray', origin='upper', vmin=0, vmax=255,
              extent=[x, x+width*resolution, y, y+height*resolution])
    for item in result['objects']:
        ax.scatter(item['x'], item['y'], color='#e53935', s=90, edgecolors='white', zorder=3)
        ax.annotate(str(item['id']), (item['x'], item['y']), xytext=(8, 8),
                    textcoords='offset points', color='#b71c1c', fontsize=14, weight='bold')
    ax.set(title=f"{result['room'].title()}: {result['count']} {result['category']}(s)",
           xlabel='Map x (m)', ylabel='Map y (m)', aspect='equal')
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
