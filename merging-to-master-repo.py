
import os
import subprocess

#project = 'CarND-PID-Control-Project'
#project = 'CarND-MPC-Project'
#project = 'CarND-Path-Planning-Project'
project = 'CarND-Semantic-Segmentation'
# shopt -s extglob &&  mv !(CarND*|README-1.md|merging-to-master-repo.py) CarND-Semantic-Segmentation


cmd1 = 'git remote add -f {st} git@github.com:ilopezfr/{st}.git'.format(st=project)
cmd2 = 'git fetch {st}  --tags'.format(st=project)
cmd3 = 'git merge {st}/master --allow-unrelated-histories -m "merging {st} to Master repo"'.format(st=project)
cmd4 = 'mkdir {st}'.format(st=project)
# shopt -s extglob && 
cmd5 = '''shopt -s extglob &&  mv !(CarND*|README-1.md|merging-to-master-repo.py) {st}'''.format(st=project)


#shopt -s extglob &&  mv !(CarND*|README-1.md|merging-to-master-repo.py) CarND-PID-Control-Project

cmd6 = 'mv .gitignore {st}'.format(st=project)

cmd7 = 'git add .'
cmd8 = 'git commit -m "Move {st} files into subdir"'.format(st=project)
cmd9 = 'git push origin master'

commands = [cmd1, cmd2, cmd3, cmd4, cmd4, cmd5]
#commands = [cmd5]
commands = [cmd6, cmd7, cmd8, cmd9]
for cmd in commands:
    print(cmd)
    #subprocess.check_call(cmd)
    os.system(cmd)

#subprocess.call(cmd2)


# import os


# command = '''
# git remote add -f {st} git@github.com:ilopezfr/{st}.git \
# && git fetch {st}  --tags \
# && git merge {st}/master --allow-unrelated-histories -m '"'"'merging {st} to Master repo'"'"'\
# && mkdir {st} \
# && shopt -s extglob && mv !(CarND*|README-1.md|merging-to-master-repo.py) {st} \
# && mv .gitignore {st} \
# && git add . \
# && git commit -m '"'"'Move {st} files into subdir'"'"' \ 
# && git push origin master
# '''

#os.system(command)

# subprocess.run(bashCommand, shell=True)

#process = subprocess.run(bashCommand.split(), stdout=subprocess.PIPE)
#output, error = process.communicate()

# import subprocess
# process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
# output, error = process.communicate()

# from subprocess import Popen, PIPE

# projects = ['CarND-PID-Control-Project'] #, 'CarND-MPC-Project']
# for project in projects:
    # string = """ git remote add -f {st} git@github.com:ilopezfr/{st}.git
    # git fetch {st} --tags """
    
    # c1 = "git remote add -f "+project+" git@github.com:ilopezfr/"+project+".git"
    # c2 = "git fetch "+project+" --tags"
    # c3 = "git merge "{st}+"/master --allow-unrelated-histories -m "merging {st} to Master repo"

    # mkdir {st}
    # shopt -s extglob
    # mv !(CarND*|README-1.md|merging-to-master-repo.py) {st}

    # mv .gitignore {st}

    # git add .
    # git commit -m "Move {st} files into subdir"

    # git push origin master;
    
    #command_list = command.format(st=project)
    #print(command_list)
    #subprocess_cmd(command_list)
    #ubprocess.run(command_list, shell=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    #return_code = subprocess.Popen(command.format(project), shell=True)

    #print(return_code)
    #int subprocess.Popen("echo %s " % user_input, stdout=PIPE).stdout.read()



    #project = "test-project"

    # cmd1 = 'git remote add -f {st} git@github.com:ilopezfr/{st}.git'.format(st=project)
    # cmd2 = 'git fetch {st}  --tags'.format(st=project)
    # cmd3 = 'git merge {st}/master --allow-unrelated-histories -m "merging {st} to Master repo"'.format(st=project)
    # cmd4 = 'mkdir {st}'.format(st=project)
    # cmd5 = 'shopt -s extglob && mv !(CarND*|README-1.md|merging-to-master-repo.py) {st}'.format(st=project)
    # cmd6 = 'mv .gitignore {st}'.format(st=project)
    # sep  = ' && '
    # command_list = [cmd1+sep+cmd2+sep+cmd3+sep+cmd4+sep+cmd5+sep+cmd6]

    # print(command_list)
    # #Popen(cmd.format(st=project)
    # process = Popen(command_list, shell=True) #, stdout=PIPE, stderr=PIPE ) # universal_newlines=True) #, stdin=PIPE,  universal_newlines=True)
    #stdout, stderr = process.communicate()

    #print(stdout)

