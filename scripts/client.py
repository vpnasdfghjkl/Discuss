import requests
import json
from itertools import islice

import numpy as np
TEST=1
def log(info):
    with open("client.txt","a") as f:
        f.write(info)

action_ports={'grab':'http://192.168.1.226:7070/grab',
         'loosen':'http://192.168.1.226:7070/loosen'}

with open("json.json","r") as j:
    action_list=json.load(j)
TEST=1
for action_n, action_detail in islice(action_list.items(),4):
    action=action_detail['action']
    param=action_detail['param']

    if  action['class'] in action_ports:
        payload = {'action':action,
                'param': param,
                'test':TEST
                }
        sel_url = action_ports[action['class']]
        response = requests.post(sel_url, json=payload)
        if response.status_code == 200:
            print(f"Action '{action['object']}' executed successfully!")
            print("Response:", response.json())
            log(f'{response.json()} successfully\n')
        else:
            print(f"Failed to execute action '{action['object']}'!")
    else:
        print(f"No matching action_port found for action '{action['object']}'")