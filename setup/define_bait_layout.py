import numpy as np
import matplotlib.pyplot as plt

# arena params
n_sites = 84
sites = np.arange(1, n_sites+1)

def bait_list(n_baits, seed):
    '''
    Generates a list of sites to bait.

    n_baits : int
        number of sites to bait
    seed : int
        for the random number generator
	'''
    # random number generator
    rng = np.random.default_rng(seed=seed)

    # generate list of baited sites
    baited_sites = np.sort(rng.choice(sites, size=n_baits, replace=False)).astype(int)

    print(f"bait {n_baits} sites: {baited_sites}")

    return baited_sites

def make_cache_map():
    '''
    Arrange site numbers according to the arena layout.
    '''
    # arrange site numbers spatially
    UL = sites[:21]
    UL = np.insert(UL, np.asarray([0, 0, 3, 3]), np.zeros(4))
    UL = UL.reshape(5, -1)

    UR = sites[21:42]
    UR = np.insert(UR, np.asarray([3, 3, 6, 6]), np.zeros(4))
    UR = UR.reshape(5, -1)

    LL = sites[42:63]
    LL = np.insert(LL, np.asarray([15, 15, 18, 18]), np.zeros(4))
    LL = LL.reshape(5, -1)

    LR = sites[63:84]
    LR = np.insert(LR, np.asarray([18, 18]), np.zeros(2))
    LR = np.append(LR, np.zeros(2))
    LR = LR.reshape(5, -1)

    # stick the quadrants together w/ rails
    upper = np.column_stack([UL, np.zeros(5), UR])
    lower = np.column_stack([LL, np.zeros(5), LR])
    cache_map = np.row_stack([upper, np.zeros(11), lower])

    return cache_map

def plot_cache_map(baited_sites):
    '''
    Plot the cache sites according to their arena layout and 
    label the baited sites.
    '''
    cache_map = make_cache_map()

    f, ax = plt.subplots(1, 1, figsize=(5, 5))
    ax.imshow(cache_map.astype(bool), cmap='gray')

    idx = np.arange(11)
    for x in idx:
        for y in idx:
            if cache_map[y, x] in baited_sites:
                ax.scatter(x, y, c='r')
            elif cache_map[y, x] == 0:
                continue
            else:
                ax.scatter(x, y, facecolors='w', edgecolors='k')

    return f, ax