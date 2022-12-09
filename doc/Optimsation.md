# Optimisation

## Wave simulation and interpolation

### Issue

There are too many copies when updating and then interpolating a wave field.

Classes involved:

`WaveSimulation`

- Calculates the x, y, z displacements at each vertex of the wave grid.
- May use internal workspace for calculations (case for FFT) in which case
  there is a redundant copy.
- For the non-FFT calculations there is no additional copy as values are
  assigned to the pass-by-ref target as they are calculated. 

`OceanTile`

- Copy from the matrixes of x, y, z displacements to a vector of vertices
  (Array3d). These are used to update the mesh used to interpolate the
  wavefield.

`TriangulatedGrid` (held by `Wavefield`)

- Copy from the updated vector of vertices into the set used in the Delauny
  triangulation used by the locate function (where we search for the triangle 
  that contains the x, y coordinate we wish to interpolate).

