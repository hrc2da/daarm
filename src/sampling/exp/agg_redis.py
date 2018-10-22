import json


def toni_to_me(config):
    archs = config[1:-1].split(',')
    fmt_arch = ''
    for elm in archs:
        if elm.strip() == 'true':
            fmt_arch += '1'
        else:
            fmt_arch += '0'
    return fmt_arch


cache = None

with open('redis_dump/dump.json') as cachefile:
    cache = json.load(cachefile)

with open('pc_data/latin_hyper_cube_samples.json') as lhc_in:
    lhc_fst = list(map(toni_to_me, json.load(lhc_in)['0']))

lhc_configs = {}
for config in lhc_fst:
    res = cache[config]
    if res != '-1,-1':
        lhc_configs[config] = res

with open('pc_data/uniform_random_samples.json') as ur_in:
    ur_fst = list(map(toni_to_me, json.load(ur_in)['0']))

ur_configs = {}
for i, config in enumerate(ur_fst):
    if i == 95:
        break
    res = cache[config]
    if res != '-1,-1':
        ur_configs[config] = res

with open('report/latin_hypercube_evaluations.json', 'w+') as lhc_dump:
    json.dump(lhc_configs, lhc_dump)

with open('report/uniform_random_evaluations.json', 'w+') as ur_dump:
    json.dump(ur_configs, ur_dump)
