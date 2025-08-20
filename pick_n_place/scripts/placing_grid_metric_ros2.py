import numpy as np
import matplotlib.pyplot as plt

# # Set random seed for reproducibility
# np.random.seed(42)

# # Define the size of the depth map grid
# H, W = 3, 3

# # Expected cloth thickness (canonical depth)
# T = 0.5  # in meters, for example

# # Create the canonical matrix (perfect flat placement)
# canonical = np.full((H, W), T)

# # Simulate an observed matrix with some deformations
# observed = canonical.copy()

# # Simulate a bump in the middle
# for i in range(H):
#     for j in range(W):
#         dist = np.sqrt((i - H/2)**2 + (j - W/2)**2)
#         observed[i, j] += 0.15 * np.exp(-dist**2 / 50)

# # Simulate a folded corner
# observed[0:5, 0:5] += 0.2

# # Add some random noise
# observed += np.random.normal(0, 0.01, size=(H, W))

# # Compute absolute error
# E = np.abs(observed - canonical)

# # Set threshold
# tau = 0.1 * T  # 10% of thickness

# # Binary correctness mask
# M = (E <= tau).astype(int)

# # Quality as percentage of correct cells
# quality = (np.sum(M) / M.size) * 100

# # Visualizations
# fig, axs = plt.subplots(1, 4, figsize=(20, 4))

# axs[0].imshow(canonical, cmap='viridis')
# axs[0].set_title("Canonical (Perfect)")

# axs[1].imshow(observed, cmap='viridis')
# axs[1].set_title("Observed (Deformed)")

# axs[2].imshow(E, cmap='hot')
# axs[2].set_title("Absolute Error")

# axs[3].imshow(M, cmap='gray')
# axs[3].set_title(f"Correctness Mask\nQuality: {quality:.2f}%")

# for ax in axs:
#     ax.axis('off')

# plt.tight_layout()
# plt.show()

# quality

def compute_quality(observed, n_div, obj_thickness):

    observed = np.array(observed)

    # Create the canonical matrix (perfect flat placement)
    canonical = np.full((n_div, n_div), obj_thickness)
    canonical = obj_thickness*np.ones(n_div*n_div)

    # Compute absolute error
    E = np.abs(observed - canonical)

    # Set threshold
    tau = 0.3 * obj_thickness  # 10% of thickness
    print("tau: ", tau)

    # Binary correctness mask
    M = (E <= tau).astype(int)

    # Quality as percentage of correct cells
    quality = (np.sum(M) / M.size) * 100

    # Visualizations
    canonical = canonical.reshape(n_div,n_div)
    observed = observed.reshape(n_div,n_div)
    M = M.reshape(n_div,n_div)
    E = E.reshape(n_div,n_div)

    # fig, axs = plt.subplots(1, 4, figsize=(20, 4))

    # axs[0].imshow(canonical, cmap='jet_r')
    # axs[0].set_title("Canonical (Perfect)")

    # axs[1].imshow(observed, cmap='jet_r')
    # axs[1].set_title("Observed (Deformed)")

    # axs[2].imshow(E, cmap='jet_r')
    # axs[2].set_title("Absolute Error")

    # axs[3].imshow(M, cmap='gray')
    # axs[3].set_title(f"Correctness Mask\nQuality: {quality:.2f}%")

    # for ax in axs:
    #     ax.axis('off')

    # plt.tight_layout()
    # plt.show()

    return quality

def hybrid(observed, n_div, obj_thickness, n_objs):

    canonical_pile_height = 0.01*n_objs

    observed = np.array(observed)

    # Create the canonical matrix (perfect flat placement)
    # canonical = np.full((n_div, n_div), canonical_pile_height)
    canonical = canonical_pile_height*np.ones(n_div*n_div)

    canonical = canonical.reshape(n_div,n_div)
    observed = observed.reshape(n_div,n_div)

    mean_error = np.mean(np.abs(observed - canonical))
    gradient = np.mean(np.abs(np.diff(observed, axis=0))) + np.mean(np.abs(np.diff(observed, axis=1))) #Gradient for roughness (horizontal + vertical differences)

    # Estimate a reasonable "max" roughness from practical data or define empirically
    max_gradient = 0.1  # a large bump or fold would result in something like this
    # max_gradient = 0.05
    # max_depth = (obj_thickness*3) + (min_depth-obj_thickness)

    # Normalize each term
    depth_score = 1 - (mean_error / canonical_pile_height)
    roughness_score = 1 - (gradient / max_gradient)  # Empirical or based on test data

    # Final weighted score
    # 4. Weighted final quality score
    alpha, beta = 0.7, 0.3  # weights
    quality = 100 * (0.7 * depth_score + 0.3 * roughness_score)

    # # Visualize
    # fig, axs = plt.subplots(1, 2, figsize=(8, 4))
    # im1 = axs[0].imshow(observed, cmap='jet_r', vmin=0.4, vmax=0.7)
    # axs[0].set_title("Observed (3x3 Grid)")
    # plt.colorbar(im1, ax=axs[0])

    # im2 = axs[1].imshow(np.abs(observed - canonical), cmap='jet_r')
    # axs[1].set_title("Error Map")
    # plt.colorbar(im2, ax=axs[1])

    # for ax in axs:
    #     ax.axis('off')

    # plt.tight_layout()
    # plt.show()

    print("canonical: ", canonical)
    print("observed: ", observed)
    print("error: ", np.abs(observed - canonical))
    print("mean error: ", mean_error)
    print("gradient: ", gradient)
    print("depth score: ", depth_score)
    print("roughness score: ", roughness_score)
    quality = 100 * (0.3 * depth_score + 0.7 * roughness_score)
    print("quality 0,3 y 0,7: ", quality)
    quality = 100 * (0.7 * depth_score + 0.3 * roughness_score)
    print("quality 0,7 y 0,3: ", quality)
    quality = 100 * (0.5 * depth_score + 0.5 * roughness_score)
    print("quality 0,5 y 0,5: ", quality)

    return quality


def placing_qual(metrics, n_div, grasp_edge_size, obj_thickness, n_objs):

    min_depth = obj_thickness * n_objs #Object/pile thickness should be 0 deformation
    # max_depth = grasp_edge_size/2 + (min_depth-obj_thickness) + 0.01 #Max depth occurs when the cloth is folded by half (grasped edge size /2) + the piled object thickness (0 for placed, obj thick for pile)
    # half_max_depth = max_depth/2
    half_max_depth = min_depth+0.01
    max_depth = (obj_thickness*3) + (min_depth-obj_thickness)
    print("min depth: ", min_depth, " / max depth: ", max_depth)

    metrics = np.array(metrics)
    flat_placement = min_depth*np.ones(n_div*n_div)
    # bad_placement = np.array([[0.05, 0.1, 0.05], [0.05, 0.1, 0.05], [0.05, 0.1, 0.05]]) #to check which is the most representative
    # bad_placement = np.array([[half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth]]) #to check which is the most representative ##for pillowcase
    # bad_placement = np.array([[max_depth, max_depth, max_depth], [max_depth, max_depth, max_depth], [max_depth, max_depth, max_depth]])
    # bad_placement = 0.16*np.ones(n_div*n_div) #for towel (system adaptability)
    bad_placement = max_depth*np.ones(n_div*n_div) #for piles of towels
    bad_placement = bad_placement.reshape(-1, 1)
    print("FLAT MATRIX: ", flat_placement)

    max_dist = np.linalg.norm(bad_placement - flat_placement, 1) #Max distance from bad placement to perfect placement (100% error) - Used for normalization
    print("BAD MATRIX: ", bad_placement)
    print("Max distance: ", max_dist)
    dist = np.linalg.norm(metrics - flat_placement, 1) #Ditance of current sample to perfect placement
    print("Distance", dist)
    # placing_error = (dist/max_dist)*100 # Normalize distance
    placing_error = (dist-min_depth)/(max_dist-min_depth)*100 # Normalize distance
    placing_quality = 100-placing_error # Get placing quality (not error)
    rospy.loginfo("Placing_quality: Placing quality %f ", placing_quality)

    return placing_quality


