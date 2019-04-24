#!/usr/bin/python
# -*- coding: utf-8 -*-

from os.path import abspath, dirname, join
from subprocess import check_output
from codecs import getdecoder


def run_ltl2ba(formula):
    script_dir = dirname(abspath(__file__))
    ltl2ba = join(script_dir, "ltl2ba.exe")
    raw_output = check_output([ltl2ba, "-f", "%s" % formula])
    ascii_decoder = getdecoder("ascii")
    (output, _) = ascii_decoder(raw_output)
    # print 'Output from ltl2ba'
    # print output
    return output

