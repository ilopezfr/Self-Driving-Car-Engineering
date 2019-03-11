

import os


bashCommand = '''
# git remote add -f CarND-Unscented-Kalman-Filter-Project git@github.com:ilopezfr/CarND-Unscented-Kalman-Filter-Project.git
# git fetch CarND-Unscented-Kalman-Filter-Project  --tags
# git merge CarND-Unscented-Kalman-Filter-Project/master --allow-unrelated-histories -m "merging CarND-Unscented-Kalman-Filter-Project to Master repo" 
# mkdir CarND-Unscented-Kalman-Filter-Project &&

# shopt -s extglob
# mv !(CarND*|README-1.md) CarND-Unscented-Kalman-Filter-Project

mv .gitignore CarND-Unscented-Kalman-Filter-Project

git add .
git commit -m "Move CarND-Unscented-Kalman-Filter-Project files into subdir"

# git push origin master;
'''

os.system(bashCommand)

# subprocess.run(bashCommand, shell=True)

#process = subprocess.run(bashCommand.split(), stdout=subprocess.PIPE)
#output, error = process.communicate()

# import subprocess
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
# output, error = process.communicate()
