import os

script_dir = os.path.dirname(__file__)
rel_path = "FinalCode.py"
abs_file_path = os.path.join(script_dir, rel_path)

file = ""

with open(abs_file_path, "r") as code:
    file += f"absuloute_dir = r'{script_dir}'\n"
    code.readline()
    file += code.read()

with open(abs_file_path, "w") as code:
    code.write(file)
    


