#!/usr/bin/env python3
"""
Generate Models
@author: Benjamin Perseghetti
@email: bperseghetti@rudislabs.com
"""
import jinja2
import argparse
import os
import numpy as np
import ast

rel_gazebo_path = ".."
rel_model_path ="../models"
script_path = os.path.realpath(__file__).replace("jinja_sensor_gen.py","")
default_env_path = os.path.relpath(os.path.join(script_path, rel_gazebo_path))
default_model_path = os.path.relpath(os.path.join(script_path, rel_model_path))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sdf_version', default="NotSet", help="SDF format version to use for interpreting model file")
    parser.add_argument('--namespace', default="NotSet", help="Namespace of robot.")
    parser.add_argument('--params', default="NotSet", help="Params dictionary.")
    args = parser.parse_args()

    if args.namespace == "":
        args.namespace = "NotSet"

    if args.params != "NotSet":
        try:
            args.params = ast.literal_eval(args.params)
        except:
            print("Failed to read passed params dictionary")
            args.params = "NotSet"
            pass

    input_filename = os.path.relpath(os.path.join(default_model_path, '{:s}/{:s}.sdf.jinja'.format(args.params["base_model"], args.params["base_model"])))
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(default_env_path))
    template_model = env.get_template(os.path.relpath(input_filename, default_env_path))

    d = {'sdf_version': args.sdf_version, \
         'params': args.params, \
         'namespace': args.namespace}

    if (not os.path.isdir('/tmp/gazebo/models/{:s}'.format(args.params["model_name"]))):
        try: 
            os.makedirs('/tmp/gazebo/models/{:s}'.format(args.params["model_name"]), exist_ok = True) 
        except OSError as error: 
            print("Directory creation error.")

    model_result = template_model.render(d)
    model_out = '/tmp/gazebo/models/{:s}/{:s}.sdf'.format(args.params["model_name"], args.params["model_name"])

    with open(model_out, 'w') as m_out:
        print(('{:s} -> {:s}'.format(input_filename, model_out)))
        m_out.write(model_result)
    
    input_config = os.path.relpath(os.path.join(default_model_path, '{:s}/model.config.jinja'.format(args.params["base_model"])))
    template_config = env.get_template(os.path.relpath(input_config, default_env_path))
    result_config = template_config.render(d)
    out_config = '/tmp/gazebo/models/{:s}/model.config'.format(args.params["model_name"])
    with open(out_config, 'w') as c_out:
        print(('{:s} -> {:s}'.format("scripts/model.config.jinja", out_config)))
        c_out.write(result_config)
