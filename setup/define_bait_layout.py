import numpy as np
import matplotlib.pyplot as plt

# arena params
n_sites = 84
sites = np.arange(1, n_sites+1)

# for plotting
feeder_x = np.asarray([0.5, 0.5, 9.5, 9.5])
feeder_y = np.asarray([0.5, 9.5, 0.5, 9.5])
feeder_col = ['xkcd:green', 'xkcd:yellow', 'xkcd:blue', 'xkcd:red']

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
def make_cache_map_col():
    '''
    Arrange site numbers according to the arena layout,
    as viewed from the bottom camera.

    Counts columnwise from left to right, as in SC cache
    sorting code.
    '''
    cache_map_flat = sites.copy()

    # insert zeros for feeders 
    feeder_zeros = np.zeros(14)
    feeder_loc = np.asarray([0, 0, 6, 6, 6, 6, 12, 12, 72, 72, 78, 78, 78, 78])
    cache_map_flat = np.insert(cache_map_flat, feeder_loc, feeder_zeros)
    cache_map_flat = np.append(cache_map_flat, np.zeros(2))
    
    # make it square
    cache_map = cache_map_flat.reshape(10, -1, order='F')
    
    # insert zeros for rail
    vert_rail = np.zeros(5)
    upper = np.column_stack([cache_map[:5, :5], vert_rail, cache_map[:5, 5:]])
    lower = np.column_stack([cache_map[5:, :5], vert_rail, cache_map[5:, 5:]])
    horiz_rail = np.zeros(11)
    cache_map = np.row_stack([upper, horiz_rail, lower])

    # h_rail_loc = np.asarray([5, 15, 25, 35, 45, 55, 65, 75, 85, 95])
    # cache_map_flat = np.insert(cache_map_flat, h_rail_loc, horiz_rail)
    return cache_map


def make_cache_map():
    '''
    Arrange site numbers according to the arena layout,
    as viewed from the bottom camera:
        UL quadrant = green feeder
        UR = blue feeder
        LL = yellow feeder
        LR = red feeder
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

def plot_cache_map(baited_sites, col_order=True, top_view=True):
    '''
    Plot the cache sites according to their arena layout and 
    label the baited sites.

    col_order :bool, default is True
        If true, numbers cache sites by column
        If false, numbers by quadrant
        
    top_view : bool, default is True
        If true, flips the map so that it matches the view from above/front of rig
        i.e., upper left corner is yellow feeder and lower right is blue.
    '''
    if col_order:
        cache_map = make_cache_map_col()
    else:
        cache_map = make_cache_map()

    # plot arena layout
    f, ax = plt.subplots(1, 1, figsize=(5, 5))
    ax.imshow(cache_map.astype(bool), cmap='gray')

    # label baited sites
    idx = np.arange(11)
    for x in idx:
        for y in idx:
            if cache_map[y, x] in baited_sites:
                ax.scatter(x, y, c='r')
            elif cache_map[y, x] == 0:
                continue
            else:
                ax.scatter(x, y, facecolors='w', edgecolors='k')
    
    # label feeders
    ax.scatter(feeder_x, feeder_y, s=200, \
                facecolors=feeder_col, edgecolors='w')

    # axes and labels
    if top_view:
        ax.set_ylim(ax.get_ylim()[::-1]) # flip view
    ax.tick_params(labelleft=False, labelbottom=False)
    ax.set_yticks([])
    ax.set_xticks([])

    return f, ax


def plot_annotated_cache_map(baited_sites, col_order=True, top_view=False):
    '''
    Plot the cache site numbers and feeder colors.

    col_order :bool, default is True
        If true, numbers cache sites by column
        If false, numbers by quadrant

    top_view : bool, default is False
        If true, flips the map so that it matches the view from above/front of rig
        i.e., upper left corner is yellow feeder and lower right is blue.
    '''
    if col_order:
        cache_map = make_cache_map_col()
    else:
        cache_map = make_cache_map()

    f, ax = plt.subplots(1, 1, figsize=(5, 5))
    ax.imshow(cache_map.astype(bool), cmap='gray')

    # number caches
    idx = np.arange(11)
    for x in idx:
        for y in idx:
            if cache_map[y, x] > 0:
                ax.text(x, y, str(cache_map[y, x].astype(int)), \
                        size=8, ha='center', weight='semibold')

    # label feeders
    ax.scatter(feeder_x, feeder_y, s=200, \
                facecolors=feeder_col, edgecolors='w')
    
    # axes and labels
    if top_view:
        ax.set_ylim(ax.get_ylim()[::-1]) # flip view
    ax.tick_params(labelleft=False, labelbottom=False)
    ax.set_yticks([])
    ax.set_xticks([])

    return f, ax