#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os

for fn in os.listdir('.'):
     if not os.path.isfile(fn):
       continue
     print os.path.basename(fn)
     os.system(format("./translator-fond/translator.py domain.pddl %s", os.path.basename(fn)))
     
  
