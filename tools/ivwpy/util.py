#*********************************************************************************
#
# Inviwo - Interactive Visualization Workshop
#
# Copyright (c) 2013-2019 Inviwo Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
#*********************************************************************************

import os
import sys
import itertools
import datetime
import math
import subprocess
import time
import re

from . import colorprint as cp
from . import util

def subDirs(path):
	if os.path.isdir(path):
		return next(os.walk(path))[1]
	else:
		return []

def toPath(*list):
	return "/".join(list)

def useForwardSlash(path):
	 return "/".join(path.split(os.sep))

def addPostfix(file, postfix):
	parts = file.split(os.path.extsep)
	parts[0]+= postfix
	return os.path.extsep.join(parts)

def in_directory(file, directory):
    #make both absolute    
    directory = os.path.join(os.path.realpath(directory), '')
    file = os.path.realpath(file)

    #return true, if the common prefix of both is equal to directory
    #e.g. /a/b/c/d.rst and directory is /a/b, the common prefix is /a/b
    return os.path.commonprefix([file, directory]) == directory

def getScriptFolder():
	import inspect
	""" Get the directory of the script is calling this function """
	return os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe().f_back))) 

def mkdir(*path):
	res = toPath(*path)	
	if not os.path.isdir(res):
		os.mkdir(res)
	return res

def partition(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i+n]

def pad_infinite(iterable, padding=None):
   return itertools.chain(iterable, itertools.repeat(padding))

def pad(iterable, size, padding=None):
   return itertools.islice(pad_infinite(iterable, padding), size)

def addMidSteps(func, iterable, transform = lambda x: x):
	''' s -> s1, func(s1,s2), s2, func(s2,s3), s3'''
	tmp = next(iterable)
	yield transform(tmp)
	for n in iterable:
		res = func(tmp, n)
		try:
			for r in res: yield r
		except TypeError:
			yield res
		tmp = n
		yield transform(n)


def makeSlice(string):
	def toInt(s):
		try:
			return int(s)
		except ValueError:
			return None

	return slice(*list(pad(map(toInt, string.split(":")), 3)))

def dateToString(date):
	return date.strftime("%Y-%m-%dT%H:%M:%S.%f")

def stringToDate(string):
	return datetime.datetime.strptime(string, "%Y-%m-%dT%H:%M:%S.%f" )

def safeget(dct, *keys, failure = None):
    for key in keys:
        if key in dct.keys():
            dct = dct[key]
        else: 
            return failure
    return dct

def find_pyconfig(path):
	while path != "":
		if os.path.exists(toPath(path, "pyconfig.ini")): 
			return toPath(path, "pyconfig.ini")
		else:
			path = os.path.split(path)[0];
	return None

def stats(l):
	mean = sum(l)/len(l)
	std = math.sqrt(sum([pow(mean-x,2) for x in l])/len(l))
	return mean, std

def openWithDefaultApp(file):
	print(file)
	if sys.platform.startswith('linux'):
	    subprocess.call(["xdg-open", file])
	elif sys.platform == "darwin":
	    subprocess.call(["open", file])
	elif sys.platform == "win32":
	    os.startfile(file)

def writeTemplateFile(newfilename, templatefilename, comment, name, define, api, incfile, author, force, verbose):
	(path, filename)  = os.path.split(newfilename)
	util.mkdir(path)

	if os.path.exists(newfilename) and not force:
		cp.print_error("... File exists: " + file + ", use --force or overwrite")
		return
	elif os.path.exists(newfilename) and force:
		cp.print_warn("... Overwriting existing file")

	#Create the template in memory
	datetimestr = time.strftime("%A, %B %d, %Y - %H:%M:%S")
	lines = []
	with open(templatefilename,'r') as f:
		for line in f:
			line = line.replace("<name>", name)
			line = line.replace("<dname>", re.sub("([a-z])([A-Z])","\g<1> \g<2>", name.replace("Kx", "")))
			line = line.replace("<lname>", name.lower())
			line = line.replace("<uname>", name.upper())
			line = line.replace("<api>", api)
			line = line.replace("<define>", define)
			line = line.replace("<incfile>", incfile)
			line = line.replace("<author>", author)
			line = line.replace("<datetime>", datetimestr)
			lines.append(line)
			if verbose: print(line, end='')

	if verbose: print("")
	finaltext = "".join(lines)

	with open(newfilename, "w") as f:
		print(comment + f.name)
		f.write(finaltext)
