Motion Library

539 animated skeleton BVHs, 34,825 frames. No character meshes.
270 paired-rig reconstructions and 269 retargeted clips.
Import files from Animations. Each JSON sidecar preserves timing and quality details.
Use Y-up, original file FPS, scale 1 initially, and enable translations on every joint.
Position-channel joints have zero OFFSET; local positions carry the skeleton geometry.
WalkCycle_review.blend is a mesh-free animated review scene in Blender Z-up coordinates.

Quality: these are approximate reconstructions. Native fidelity is unverified.
All clips passed Blender import/export checks against reconstructed poses.
Eight clips omit a partial final interval, at most 0.025336 seconds.
Some clips contain rapid rotations retained from their animation curves.
This package does not assert authorship or ownership of the underlying motions.
