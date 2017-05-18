#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pdb

def show(d):
    plt.plot(d)
    plt.show()


def t_np(n):
    a = np.arange(5*n).reshape(n,5) 
    print "a.shap=",a.shape
    print "a.size=",a.size

if __name__ == '__main__':
    x = 1
    print 'x=' ,x

    y = range(10)
    pdb.set_trace()
    show(y)
    for d in y:
        print 'd=',d
    t_np(6)
   
