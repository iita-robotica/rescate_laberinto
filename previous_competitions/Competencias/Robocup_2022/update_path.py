#!! run after installing repo

# Midifes "main.py" to add the actual directory to sys.path

import os

script_dir = os.path.dirname(__file__)
rel_path = "refactored_code/run.py"
abs_file_path = os.path.join(script_dir, rel_path)

file = ""

with open(abs_file_path, "r") as code:
    file += f"absuloute_dir = r'{script_dir}'\n"
    code.readline()
    file += code.read()

with open(abs_file_path, "w") as code:
    code.write(file)
    


