import random
import os.path as path
import numpy as np # arrays
import matplotlib.pyplot as plt # plotting

path2file =  path.abspath(path.join(__file__ ,"../data/states_sample.txt"))
#print path2file

# read files into numpy arrays
## read centralized solutions
f = np.genfromtxt("../data/CentralizedSolutions.txt",delimiter=",", dtype=int)
centrSol = np.delete(f,-1,1) # Clean: removes last column of commas
f = None
## read local solutions
f = np.genfromtxt("../data/LocalSolutions.txt",delimiter=",", dtype=int)
localSol = np.delete(f,-1,1)
f = None
## read local neighbors
f = np.genfromtxt("../data/LocalNeighborsCount.txt",delimiter=",", dtype=int)
localNeighors = np.delete(f,-1,1)
f = None
## read local attackers
f = np.genfromtxt("../data/LocalAttackersCount.txt",delimiter=",", dtype=int)
localAttackers = np.delete(f,-1,1)
f = None

print '(Number od samples, number of agents) from all read files'
print centrSol.shape
print localAttackers.shape
print localSol.shape
print localNeighors.shape

# predefine number of agents in each team
Nd, Ne = 5,5

# number of read samples & total number of agents
(Nsamples, Na) = centrSol.shape

mismatchN= range(0,Nd)
percentage = [0]*Nd
# find percentage of exact match among all samples
counter = 0.
for s in range(0,Nsamples):
	# get centralized vs. local solutions
	cent = centrSol[s,:]
	local = localSol[s,:]
	#print cent, '--', local
	if (sum(cent == local) > (Nd-1)):
		counter = counter +1.
# %
print 'percentage of exact match = ', counter/Nsamples*100.
percentage[0] = counter/Nsamples*100.

# find different mismatch %
for i in range(1,Nd):
	# find percentage of i mismatch among all samples
	counter = 0.
	for s in range(0,Nsamples):
		# get centralized vs. local solutions
		cent = centrSol[s,:]
		local = localSol[s,:]
		if (sum(cent == local) > (Nd-1 -i)):
			counter = counter +1.
	# get %
	print 'percentage of' , i,' mismatch match = ', counter/Nsamples*100.
	percentage[i]=counter/Nsamples*100.
fig = plt.figure()
fig.suptitle('Centralized Vs. Local solutions\n(5 vs. 5) in 10x10 grid', fontsize=14, fontweight='bold')
ax = fig.add_subplot(111)
fig.subplots_adjust(top=0.85)
ax.set_title('Mismatch percentage')

ax.set_xlabel('Number of Mismatch')
ax.set_ylabel('Percentage %')

ax.plot(mismatchN, percentage, 'ro')
ax.axis([0, 6, 0, 100])

plt.show()



