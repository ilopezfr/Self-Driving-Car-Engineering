

import os


bashCommand = '''
git remote add -f CarND-Kidnapped-Vehicle-Project git@github.com:ilopezfr/CarND-Kidnapped-Vehicle-Project.git
git fetch CarND-Kidnapped-Vehicle-Project  --tags
git merge CarND-Kidnapped-Vehicle-Project/master --allow-unrelated-histories -m "merging CarND-Kidnapped-Vehicle-Project to Master repo"

mkdir CarND-Kidnapped-Vehicle-Project
shopt -s extglob
mv !(CarND*|README-1.md|merging-to-master-repo.py) CarND-Kidnapped-Vehicle-Project

mv .gitignore CarND-Kidnapped-Vehicle-Project

git add .
git commit -m "Move CarND-Kidnapped-Vehicle-Project files into subdir"

git push origin master;
'''

os.system(bashCommand)

# subprocess.run(bashCommand, shell=True)

#process = subprocess.run(bashCommand.split(), stdout=subprocess.PIPE)
#output, error = process.communicate()

# import subprocess
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
# output, error = process.communicate()
