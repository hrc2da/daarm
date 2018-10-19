from surrogate_sampling import sampleLatinHypercube, sampleUniformRandom, sampleOrthoganalArrays
import json


def listifySamples(samples):
    listified = []
    for sample in samples:
        listified.append(str([bool(int(i)) for i in sample]).lower())
    return listified


numSamplings = 1
sample_size = 100

lh_samples = {}
for i in range(numSamplings):
    curr_sample = sampleLatinHypercube(sample_size)
    lh_samples[i] = listifySamples(curr_sample)

with open("data/latin_hyper_cube_samples_init.json", "w+") as outfile:
    json.dump(lh_samples, outfile)

ur_samples = {}
for i in range(numSamplings):
    curr_sample = sampleUniformRandom(sample_size)
    ur_samples[i] = listifySamples(curr_sample)

with open("data/uniform_random_samples_init.json", "w+") as outfile:
    json.dump(ur_samples, outfile)
