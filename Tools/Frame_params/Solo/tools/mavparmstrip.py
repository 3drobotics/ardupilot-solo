#!/usr/bin/python
import re
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('--strip', metavar='STRIP_FILE')
parser.add_argument('master', metavar='MASTER_FILE')
parser.add_argument('output', metavar='OUTPUT_FILE', nargs='?')

args = parser.parse_args()

param_regex = re.compile("^\s*([A-Za-z0-9_]+)\s*,{0,1}\s*([-+]?[0-9]*\.?[0-9]+)")

params = {}

f_master = open(args.master, 'r')
for line in f_master:
    match = param_regex.match(line)
    if match is None:
        continue
    name,value = match.groups()
    value = float(value)
    params[name] = value
f_master.close()

if args.strip is not None:
    f_strip = open(args.strip, 'r')
    for line in f_strip:
        strip_regex = re.compile(line.strip())
        for (name,value) in params.items():
            if strip_regex.match(name):
                del params[name]
    f_strip.close()

if args.output is not None:
    f_output = open(args.output, 'w')
    for (name,value) in sorted(params.items()):
        f_output.write("%s, %f\n" % (name, value))
    f_output.truncate()
    f_output.close()
else:
    for (name,value) in sorted(params.items()):
        print("%s, %f" % (name, value))
