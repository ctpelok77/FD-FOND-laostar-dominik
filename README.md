### LAO* search based FOND Planning Framework  ###
* Version: 0.1

* Framework used in publication *Structural Symmetries for Fully Observable 
Nondeterministic Planning* [[pdf]](https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=0ahUKEwjWzou1j_HQAhWG6xoKHYTaDD4QFggjMAA&url=http%3A%2F%2Fai.cs.unibas.ch%2Fpapers%2Fwinterer-et-al-ijcai2016.pdf&usg=AFQjCNH859jpNw9nQDCWbbC-sfEbFG7kew&sig2=muBTClNm8K-0YjTLf_aBzg&bvm=bv.141320020,d.d2s) [[bib]](http://dblp.uni-trier.de/rec/bibtex/conf/ijcai/WintererW016)

```
### Required Software ###
python, cmake, libc6-dev-i386, libx32gcc-4.8-dev  

### Obtaining the Framework ###
hg clone https://wintered@bitbucket.org/wintered/laostarsearch laostar

### Build ###:
cd src/
./build_all

### Sample usage ###:
LAO* using FF heuristic (based on the all-outcome determinization of the problem)
./src/fast-downward.py --alias ff benchmarks/blocksworld/p10.pddl

LAO* using FF blind heuristic and Symmetry Reduction
./src/fast-downward.py --alias ff-oss benchmarks/blocksworld/p10.pddl

LAO* using blind heuristic
./src/fast-downward.py --alias blind benchmarks/blocksworld/p10.pddl

```
### Licence ###
* The translator is an adaptation of the Fast Downward translator to FOND planning by Christian Muise (see [Planner for Relevant Policies](https://bitbucket.org/haz/planner-for-relevant-policies/wiki/Home)), provided as is, without any change.
* The search framework is based on the [Fast Downward](http://www.fast-downward.org/) framework, adapted to handle multiple effects. For the unchanged parts the licence of Fast Downward applies.
* The symmetries framework is based on the framework for the classical planning by Carmel Domshlak, Michael Katz, and Alexander Shleyfman.
* For all other parts:  Copyright (C) 2015-2016 Dominik Winterer, Martin Wehrle, Michael Katz.  The code is released under the [GNU General Public License](https://www.gnu.org/licenses/#GPL), version 3 or later.

LAO* search based FOND Planning Framework is free software: you can redistribute it and/or modify it under  the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

LAO* search based FOND Planning Framework  is distributed in the hope that it will be  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.        
                                                                                 
You should have received a copy of the GNU General Public License along with    
this program. If not, see <http://www.gnu.org/licenses/>.

###  Issues? Questions? ###
Feel free to write an email to dominik_winterer@gmx.de