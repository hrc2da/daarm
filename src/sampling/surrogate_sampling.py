import lhsmdu
import matplotlib.pyplot as plt
import numpy as np
import argparse
import json
from random import random


def sampleLatinHypercube(sampleSize, show_graph=False):
    samples = np.array(lhsmdu.sample(60, sampleSize)).T
    for i in range(sampleSize):
        for j in range(60):
            if samples[i, j] > 0.5:
                samples[i, j] = 1
            else:
                samples[i, j] = 0
    if show_graph:
        feature_representation = np.sum(samples, axis=0)
        plt.bar([i for i in range(1, 61)], feature_representation)
        plt.show()
    return samples


def sampleOrthoganalArrays(sampleSize, show_graph=False):
    return


def sampleUniformRandom(sampleSize, show_graph=False):
    samples = np.zeros((60, sampleSize)).T
    for i in range(sampleSize):
        for j in range(60):
            if random() > 0.5:
                samples[i, j] = 1
            else:
                samples[i, j] = 0
    if show_graph:
        feature_representation = np.sum(samples, axis=1)
        plt.bar([i for i in range(1, 61)], feature_representation)
        plt.show()
    return samples


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Sampling methods for EOSS')
    parser.add_argument('sampling_method', metavar='m', type=str,
                        help='Sampling method to use')
    parser.add_argument('num_samples', metavar='n', type=int,
                        help='Number of samples to collect')
    parser.add_argument('show_graph', metavar='g', type=bool, help='whether to show graph or not')
    args = parser.parse_args()
    if args.sampling_method in ['lh']:
        sampleLatinHypercube(args.num_samples, show_graph=args.show_graph)
    if args.sampling_method in ['oa']:
        sampleOrthoganalArrays(args.num_samples, show_graph=args.show_graph)
    if args.sampling_method in ['ur']:
        sampleUniformRandom(args.num_samples, show_graph=args.show_graph)
