clc;clear all;close all;

G = digraph;
s = [1 1];
t = [2 3];
r = [1 3 4 5 6 3 4];
G = addedge(G,s,t);
G = addedge(G,1,4);
st0 = char('P'+string(0));
st1 = char('P'+string(1));
st2 = char('P'+string(2));
G.Nodes.path(1) = {r};
plot(G)
