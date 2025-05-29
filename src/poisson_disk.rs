use core::f32;

use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use rand::rngs::StdRng;
use rand::seq::index::sample;
use rand::SeedableRng;

pub fn even_spaced_indices(points: &[[f32; 2]], n_samples: usize, rng_seed: u64) -> Vec<usize> {
    // Select indices of points that are evenly spaced in the 2D plane.
    //
    // Uses Poisson Disk Sub-sampling to select points that are at least a certain distance apart.
    //
    // Arguments:
    //     points: 2D array of points, shape (n, 2).
    //     n_samples: Number of samples to select.
    //     rng_seed: Seed for random number generator.
    //
    // Returns:
    //     A vector of indices of the selected points, length n_samples.

    // Approximate boinding box area covered by points
    let min_x = points.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min);
    let max_x = points
        .iter()
        .map(|p| p[0])
        .fold(f32::NEG_INFINITY, f32::max);
    let min_y = points.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
    let max_y = points
        .iter()
        .map(|p| p[1])
        .fold(f32::NEG_INFINITY, f32::max);
    let area = (max_x - min_x) * (max_y - min_y);

    // Approximate radius that will result in n_samples evenly spaced points
    let r = f32::sqrt(area / n_samples as f32 / 2.5);
    let r2 = r * r;

    let mut accepted_indices = Vec::with_capacity(n_samples);
    let mut kdtree: KdTree<f32, usize, [f32; 2]> = KdTree::new(2);

    // Create ramdom sample iterator to select points
    // The maximum number of samples is 10 times the requested number of samples to limit the number of iterations
    let mut rng = StdRng::seed_from_u64(rng_seed);
    let permutated_indices = sample(&mut rng, points.len(), n_samples * 10);

    // Perform Poisson disk sampling
    for index in permutated_indices {
        let point = points[index];

        // Check if the point is too close to an existing point in the kdtree
        if let Some((squared_dist, _)) = kdtree
            .iter_nearest(&point, &squared_euclidean)
            .unwrap()
            .next()
        {
            if squared_dist < r2 {
                continue; // too close to an existing point
            }
        }

        // Add the point to the accepted indices and kdtree
        accepted_indices.push(index);
        kdtree.add(point, index).unwrap();

        // Stop if we have enough samples
        if accepted_indices.len() >= n_samples {
            break;
        }
    }

    accepted_indices
}
