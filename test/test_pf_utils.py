import math
import numpy as np
import sys
from pathlib import Path

# --- ensure local import works ---
sys.path.append(str(Path(__file__).resolve().parents[2] / "robot_localization"))

from robot_localization.pf import Particle, ParticleFilter  # now works even without ROS env

def test_particle_initialization():
    """Check that a Particle stores x, y, theta, w correctly."""
    p = Particle(1.0, 2.0, math.pi / 4, 0.5)
    assert p.x == 1.0
    assert p.y == 2.0
    assert abs(p.theta - math.pi / 4) < 1e-6
    assert p.w == 0.5

def test_normalize_particles():
    """Check that particle weights sum to 1 after normalization."""
    pf = ParticleFilter()
    pf.particle_cloud = [
        Particle(0, 0, 0, 1.0),
        Particle(1, 1, 0, 2.0),
        Particle(2, 2, 0, 3.0),
    ]
    pf.normalize_particles()
    total = sum(p.w for p in pf.particle_cloud)
    assert abs(total - 1.0) < 1e-6

def test_resample_preserves_count():
    """Check that resampling returns the same number of particles."""
    pf = ParticleFilter()
    pf.particle_cloud = [Particle(np.random.rand(), np.random.rand(), 0, 1.0) for _ in range(100)]
    pf.normalize_particles()
    pf.resample_particles()
    assert len(pf.particle_cloud) == pf.n_particles
