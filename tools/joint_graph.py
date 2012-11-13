#!/usr/bin/env python

# Parse an sdf or urdf file and generate a joint graph using graphviz

from xml.dom import minidom
import argparse

parser = argparse.ArgumentParser(
	description='Generate a joint graph from an sdf or urdf file')
parser.add_argument('-t', dest='graphTitle', nargs=1, default='graph',
	help='title of graph')
parser.add_argument('-i', dest='inFile', nargs=1, required=True,
	help='name of sdf/urdf input file')
parser.add_argument('-o', dest='outFile', nargs=1, required=True,
	help='name of pdf joint graph output file')
parser.add_argument('-u', '--undirected', action='store_true',
	help='generate undirected graph, otherwise generate directed graph')


args = parser.parse_args()
#print args.inFile[0]
#print args.outFile[0]

if args.undirected:
	graphKeyword = "graph"
	graphConnector = " -- "
else:
	graphKeyword = "digraph"
	graphConnector = " -> "

xmldoc = minidom.parse(args.inFile[0])

joints = xmldoc.getElementsByTagName('joint')

print graphKeyword + args.graphTitle[0] + " {"
for j in joints:
	jp = j.getElementsByTagName('parent')
	jc = j.getElementsByTagName('child')
	if j.parentNode.tagName == 'robot':
		if len(jp) == 1:
			jp0 = jp[0]
			if len(jc) == 1:
				jc0 = jc[0]
				print "\t" + jp0.getAttribute('link') + graphConnector + jc0.getAttribute('link') + ";"
	if j.parentNode.tagName == 'model':
		if len(jp) == 1:
			jp0 = jp[0]
			jp00 = jp0.childNodes[0]
			if len(jc) == 1:
				jc0 = jc[0]
				jc00 = jc0.childNodes[0]
				print "\t" + jp00.nodeValue + graphConnector + jc00.nodeValue + ";"
print "}"

