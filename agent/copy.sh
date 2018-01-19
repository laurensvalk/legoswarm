#!/usr/bin/env bash
# quick and dirty script to push changed files to robot, now that we can't use stretch & vscode
remote=/run/user/1000/gvfs/sftp:host=10.42.0.113/home/robot
list="$(find *.py -mmin -60 -type f | xargs ls)"
for item in $list
do
	echo "copying $item"
	cp "$item" $remote
done

