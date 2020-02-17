import fnmatch
import os
import re
import codecs
import sys

try:
	import colorama
	colorama.init()
	
	def print_error(mess, **kwargs):
		print(colorama.Fore.RED + colorama.Style.BRIGHT + mess + colorama.Style.RESET_ALL, **kwargs)
	def print_warn(mess, **kwargs):
		print(colorama.Fore.YELLOW + colorama.Style.BRIGHT + mess + colorama.Style.RESET_ALL, **kwargs)
	
except ImportError:
	def print_error(mess, **kwargs):
		print(mess, **kwargs)
	def print_warn(mess, **kwargs):
		print(mess, **kwargs)

def exclude_name(name, filters):
	for filter in filters:
		if fnmatch.fnmatch(name, filter):
			return True
	return False


def find_files(paths, extensions, excludes=[""]):
	matches = []
	for path in paths:
		for root, dirnames, filenames in os.walk(path):
			dirnames[:] = [d for d in dirnames if d not in [".git"]]
			for extension in extensions:
				for filename in fnmatch.filter(filenames, extension):
					if not exclude_name(root + os.sep + filename, excludes):
						matches.append(os.path.join(root, filename))
	return matches

def find_matches(files, expr):
	r = re.compile(expr)
	matches = []
	for file in files:
		match_in_file = False
		with codecs.open(file, 'r', encoding="UTF-8") as f:
			try:
				for (i,line) in enumerate(f):
					match = r.search(line) 
					if match:
						if not match_in_file: 
							print_warn("Match in: " + file)
							match_in_file = True
						matched = colorama.Fore.YELLOW + match.group(0) + colorama.Style.RESET_ALL
						print("{0:5d} {1:s}".format(i,matched))
			except UnicodeDecodeError:
				print_error("Encoding error: " + file)

		if match_in_file:
			matches.append(file)
	return matches

def replace_matches(files, expr, repl, dummy=False):
	r = re.compile(expr)
	for file in files:
		match_in_file = False
		
		with codecs.open(file, 'r', encoding="UTF-8") as f:
			try: 
				lines = f.readlines()
			except UnicodeDecodeError:
				print_error("Encoding error: " + file) 
				continue

		if not dummy:
			with codecs.open(file, 'w', encoding="UTF-8") as f:
				for (i,line) in enumerate(lines):
					match = r.search(line)
					if match:
						if not match_in_file: 
							print_warn("Match in: " + file)
							match_in_file = True
						matched = colorama.Fore.YELLOW + match.group(0) + colorama.Style.RESET_ALL
						replaced = r.sub(colorama.Fore.RED + repl + colorama.Style.RESET_ALL, line.rstrip())
						f.write(re.sub(r, repl, line))
						print("- {0:5d} {1:s}".format(i,matched))
						print("+ {0:5d} {1:s}".format(i,replaced))
					else:
						f.write(line)
		else: 
			for (i,line) in enumerate(lines):
				match = r.search(line)
				if match:
					if not match_in_file: 
						print_warn("Match in: " + file)
						match_in_file = True
					matched = colorama.Fore.YELLOW + match.group(0) + colorama.Style.RESET_ALL
					replaced = r.sub(colorama.Fore.RED + repl + colorama.Style.RESET_ALL, line.rstrip())
					print("- {0:5d} {1:s}".format(i,matched))
					print("+ {0:5d} {1:s}".format(i,replaced))


def check_file_type(files, enc):
	matches = []
	for f in files:
		try:
			fh = codecs.open(f, 'r', encoding=enc)
			fh.readlines()
			fh.seek(0)
		except UnicodeDecodeError:
			print_warn("Encoding error: " + f)
			matches.append(f)

	return matches

def convert_file(file, enc):
	with open(file, 'br') as f:
		buff = f.read()
	text = buff.decode(enc, errors='replace')
	with open(file, 'w') as f:
		f.write(text)

source_extensions = ['*.cpp', '*.h']

# examples
# in ipython "run tools/replace-in-files.py"
# n = find_files(["."], source_extensions, excludes=["*ext*"])
# n = find_files(["inviwo-dev", "inviwo-research"], source_extensions, excludes=["*ext*", "*moc*", "*cmake*"])
# n = find_files(["inviwo-dev", "inviwo-research"], source_extensions, excludes=["*/ext/*", "*moc_*", "*cmake*", "*/proteindocking/*", "*/proteindocking2/*", "*/genetree/*"])
# replace_matches(n, r"^(.*)ProcessorClassIdentifier\((\S+)\s*,\s*(.+)\);?\W*$", r"\1\2ProcessorClassVersion(\3, 0);\n")


