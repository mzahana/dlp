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
exact_match_ind=[] # indices of smaples correspond ot exact match
# find percentage of exact match among all samples
counter = 0.
for s in range(0,Nsamples):
	# get centralized vs. local solutions
	cent = centrSol[s,:]
	local = localSol[s,:]
	#print cent, '--', local
	if (sum(cent == local) > (Nd-1)):
		exact_match_ind.append(s)
		counter = counter +1.
# %
print 'percentage of exact match = ', counter/Nsamples*100.
percentage[0] = counter/Nsamples*100.

# neighborhood  matrices
Nh_d = []
Nh_a = []
for i in range(0,Nd):
	Nh_d.append([0]*int(counter))
	Nh_a.append([0]*int(counter))

for i in range(0,Nd):
	for j in range(0,int(counter)):
		Nh_d[i][j]=localNeighors[exact_match_ind[j], i]
		Nh_a[i][j]=localAttackers[exact_match_ind[j], i]

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
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
fig.subplots_adjust(top=0.85)
ax1.set_title('Mismatch percentage')

ax1.set_xlabel('Number of Mismatch')
ax1.set_ylabel('Percentage %')

ax1.plot(mismatchN, percentage, 'ro')
ax1.axis([0, 6, 0, 100])

#ax2
ax2.set_title('Neighborhood size(defenders) for exact match')
for p in range(0,Nd):
	ax2.plot(Nh_d[p][:],'o')
ax2.set_xlabel('sample number')
ax2.set_ylabel('number of local defenders')

#ax3
ax3.set_title('Neighborhood size(attackers) for exact match')
for p in range(0,Nd):
	ax3.plot(Nh_a[p][:],'o')
ax3.set_xlabel('sample number')
ax3.set_ylabel('number of local attackers')

plt.show()



