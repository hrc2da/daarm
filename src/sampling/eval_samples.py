import redis
import requests
import json

EVAL_URI = "https://www.selva-research.com/api/vassar/evaluate-architecture"
SESSION_URI = "https://www.selva-research.com/api/daphne/set-problem"
cached_evals = redis.Redis(host="localhost", port=6379)
vassar_session = requests.session()
vassar_session.post(SESSION_URI, json={"problem": "ClimateCentric"})


def eval_config(config):
    response = vassar_session.post(EVAL_URI, json={"special": "False", "inputs": config})
    result = str(response.json()["outputs"])[1:-1]  # weird thing to make the result a comma-sep string for consistency
    science, cost = map(float, result.split(','))
    return science, cost


fp = 'data/latin_hyper_cube_samples_init.json'
config_eval = {}
data = {}
with open(fp, 'r+') as infile:
    data = json.load(infile)
    config_eval = {}
    for sampling_iter, config_list in data.items():
        for sample, config in enumerate(config_list):
            print("Evaluating config #%s : %s" % (sample, config))
            config_eval[config] = eval_config(config)
            print("Eval result: %s , %s" % config_eval[config])

with open('lhc_eval.json', 'w+') as outfile:
    json.dump(config_eval, outfile)
