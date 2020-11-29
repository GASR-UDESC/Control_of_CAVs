from machine import automata
from machine import operations
from machine import dijkstra2 as dk
from machine import rob_callback as rc
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np

#Creating States
A = automata.State('A')
B = automata.State('B')
C = automata.State('C')
D = automata.State('D')
E = automata.State('E')
F = automata.State('F')

# Creating events
a = automata.Event('a', 1, True)
b = automata.Event('b', 1, True)
c = automata.Event('c', 1, True)
d = automata.Event('d', 1, True)
e = automata.Event('e', 1, True)
f = automata.Event('f', 1, True)
g = automata.Event('g', 1, True)
h = automata.Event('h', 1, True)
k = automata.Event('k', 1, True)
l = automata.Event('l', 1, True)
m = automata.Event('m', 1, True)
n = automata.Event('n', 1, True)
o = automata.Event('o', 1, True)
p = automata.Event('p', 1, True)

#Creating the automaton itself
transitions = {A: {a: B, b: D}, B: {c: A, d: C, e: E}, C: {f: B, g: F}, D: {h: A, k: E}, E: {l: B, m: D, n: F}, F: {o: C, p: E}}
G = automata.Automaton(transitions, A)

G.remove_transitions(a)
print(G.transitions)