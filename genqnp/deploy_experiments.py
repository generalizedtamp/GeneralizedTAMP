import yaml
import time
import subprocess
import os
from utils import get_arg_string, get_experiment_dict
import shutil

assert os.environ['S3ACCESS_CUST']
assert os.environ['S3SECRET_CUST']

# Set up the hyperparameters here 
num_seeds = 6
run_name = "realrun10"

for mode in ["gentamp", "standard", "skeleton"]:
    # for exp_name in ["stack3d_0", "sort_0", "carry_0", "clutter_0"]:
    for exp_name in ["clutter_0"]:
        with open("generalized_tamp.yaml", 'r') as stream:
            try:
                conf = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        exp_dict = get_experiment_dict(exp_name)

        for testing_name in exp_dict['testing_problems']:
            for seed in range(num_seeds):
                setup_name = str(time.time())
                args_env = {"mode": mode,
                            "exp_name": exp_name,
                            "test_problem": testing_name,
                            "run_name": run_name,
                            "seed": seed,
                            "S3ACCESS_CUST": os.environ['S3ACCESS_CUST'],
                            "S3SECRET_CUST": os.environ['S3SECRET_CUST']}
                converted_env = []
                for args_env_keys, args_env_values in args_env.items():
                    converted_env.append({"name": args_env_keys, "value": str(args_env_values)})

                conf['spec']['template']['spec']['containers'][0]['env'] = converted_env
                conf['metadata']['name'] = "spatial-planning-"+str(setup_name)

                dirpath = "./temp"
                if os.path.exists(dirpath) and os.path.isdir(dirpath):
                    shutil.rmtree(dirpath)

                os.mkdir(dirpath)
                conf_filename = './temp/tmp_delpoyment_conf_'+setup_name+'.yml'
                with open(conf_filename, 'w') as outfile:
                    yaml.dump(conf, outfile, default_flow_style=False)

                # Deploy
                print("Deploying "+str()+"...")
                subprocess.run(["kubectl", "apply", "-f", os.getcwd()+"/"+str(conf_filename)] )
                print("Deployed")
