import random
import os.path as path

# game configuration
Nd = 5						# N of defenders
Na = 5 						# N of attackers
Nagents = Nd+Na		# total number of agents

ns = 10*10				# number of sectors

Nsamples = 10**5	# number of random samples

# test writing to file
path2file =  path.abspath(path.join(__file__ ,"../data/states_sample.txt"))
print path2file
o_file = open('../data/states_sample.txt', 'w')
# TODO: write header

 
# Main loop

for x in range(1, Nsamples+1):
	# generates a sequence of random numbers of length =  Nagents
	# first Nd correspond to defenders sectors; remaining are for attackers.
	s = random.sample(xrange(1, ns), Nagents)
	for i in s:
		o_file.write('%s,' % i)
	o_file.write('\n')

o_file.close()
