#!/usr/bin/env python3
"""Render a saved course occupancy map and actual search result as a PNG."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/semantic_search'))
from semantic_search.map_resources import resolve_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result')
    parser.add_argument('--map', type=Path, help='Explicit map version; default is the registered baseline map')
    parser.add_argument('--output', default='results/validation/semantic_map.png')
    args = parser.parse_args()
    result = json.loads(Path(args.result).read_text())
    resource = resolve_map(ROOT, yaml_path=args.map) if args.map else resolve_map(ROOT, map_id='baseline_v1')
    info = resource['metadata']
    if abs(info['origin'][2]) > 1e-6:
        raise ValueError('PNG preview expects the axis-aligned course map; use semantic_map.svg for rotated maps')
    image = plt.imread(resource['image'])
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
