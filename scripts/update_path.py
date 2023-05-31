#run after installing repo!!

# Modifes "run.py" to add the actual directory to sys.path

import os
import pathlib

script_dir = pathlib.Path(os.path.abspath(__file__)).parent
rel_path = pathlib.Path("src/run.py")

abs_target_file_path = pathlib.Path(script_dir.parent, rel_path)

file = ""

with open(abs_target_file_path, "r") as code:
    file += f"absuloute_dir = r'{abs_target_file_path.parent}'\n"
    code.readline()
    file += code.read()

with open(abs_target_file_path, "w") as code:
    code.write(file)
    


