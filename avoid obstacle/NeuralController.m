function [ML, MR] = NeuralController(LS, RS,w1,w2,w3,w4)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

ML = (LS*w1+RS*w3) > 3.506; 
MR = (RS*w4+LS*w2) > -1.775;


end