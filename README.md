# Projective Displacement Mappping

<img width=80% src="https://github.com/quantasci/ProjectiveDisplacement/blob/main/docs/pdm_teaser2.jpg" />

Reference CPU implementation for the paper:<br>
2025, Hoetzlein, <a href="http://dx.doi.org/10.1111/cgf.70235">Projective Displacement Mapping for Ray Traced Editable Surfaces</a><br>
Pacific Graphics 2025. Taipei, Taiwan.

<img width=80% src="https://ramakarl.com/imgs/PG2025_Best_Paper.png" />

Project Website: <a href="https://ramakarl.com/pdm">https://ramakarl.com/pdm</a><br>
Full video (narrated): <a href="https://www.youtube.com/watch?v=2OcRSXS_dmQ">Youtube</a><br>
Appendices & Proofs: <a href="https://arxiv.org/abs/2502.02011">arXiv</a><br>

## How to Build
**Updated Nov 2025**<br>
Build with cmake is now simpler and faster.<br>
Libmin dependent code is directly compiled into the project (no shared or static libs).<br>
Steps:<br>
1. Clone this ProjectiveDisplacement repo<br>
2. Clone <a href="https://github.com/ramakarl/libmin">libmin</a> as a sibling folder<br>
<pre>
\codes
 ├── \ProjectiveDisplacement
 └── \libmin
</pre>
3. Run cmake or cmake-gui on the ProjectiveDisplacement cmake.<br>

See <a href="https://github.com/ramakarl/libmin">libmin</a> for more details.

## CPU Implementation
This repository contains the reference CPU implementation for Projective Displacement Mapping,
for algorithm validation and correctness. No BVH, acceleration structures, or multi-core.<br>
Details can be found in proj_displace_mesh.cpp.<br>
The entry point for the primary algorithm is the function: RaytraceDisplacementMesh

## GPU Implementation
For GPU Implementation, contact me at: <a href="mailto:ramahoetzlein@gmail.com">ramahoetzlein@gmail.com</a>

## License
Rama Hoetzlein (c) Quanta Sciences, 2023-2025. <br>
MIT License

<img width=80% src="https://github.com/quantasci/ProjectiveDisplacement/blob/main/docs/pdm_teaser1.jpg" />
