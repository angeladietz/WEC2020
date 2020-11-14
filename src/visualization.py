import os

import matplotlib as mpl
import matplotlib.pyplot as plt

from .simulator import Simulator


def visualize_single_frame(sim):
    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.pcolormesh(sim.contamination, ec='lightgrey', lw=1,
                   cmap='Wistia', norm=mpl.colors.Normalize(0, 99), zorder=1)

    for i in range(sim.shape[0]):
        for j in range(sim.shape[1]):
            ax1.text(j + 0.5, i + 0.5, int(sim.contamination[i, j]),
                     fontsize=15, ha='center', va='center', zorder=2)

    xmin = 0
    xmax = sim.shape[0]
    ymin = 0
    ymax = sim.shape[1]

    for s in list(sim.stations):
        xmin = min(xmin, s[0])
        xmax = max(xmax, s[0] + 1)
        ymin = min(ymin, s[1])
        ymax = max(ymax, s[1] + 1)
        rect = mpl.patches.Rectangle((s[1], s[0]), 1, 1,
                                     color='green', ec='lightgrey', lw=1, zorder=2)
        ax1.add_patch(rect)

    legend_patches = []
    legend_labels = []

    cmap = mpl.cm.get_cmap('terrain')
    num_robots = len(sim.robots)

    for i, (name, r) in enumerate(sim.robots.items()):
        circ = mpl.patches.Circle((r.pos[1] + 0.5, r.pos[0] + 0.5), 0.3,
                                  color=cmap(i / num_robots), ec='darkgrey', lw=1, zorder=3)
        ax1.add_patch(circ)
        legend_patches.append(circ)
        legend_labels.append(f'{name}: fuel={r.fuel}, fluid={r.fluid}')

    ax1.set_aspect('equal')
    ax1.set_xticks([])
    ax1.set_yticks([])
    ax1.set_xlim(ymin - 1, ymax + 1)
    ax1.set_ylim(xmax + 1, xmin - 1)

    ax2.axis('off')
    ax2.legend(legend_patches, legend_labels, loc='center')

    return fig


def visualize_everything(prob, soln, output_dir):
    sim = Simulator(prob, soln.robots)

    fig = visualize_single_frame(sim)
    fig.savefig(os.path.join(output_dir, '0.png'))
    plt.close(fig)

    for i, action in enumerate(soln.actions):
        print(f'Generating image for frame {i + 1}...')
        sim.apply(action)
        fig = visualize_single_frame(sim)
        fig.savefig(os.path.join(output_dir, f'{i + 1}.png'))
        plt.close(fig)

    print('All images generated!')
