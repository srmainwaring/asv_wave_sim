# Nonlinear Buoyancy and Froude-Krylov Forces

---

Author: Rhys Mainwaring\
Date: 25 October 2022

---

This note provides some insight into the buoyancy calculation used in the
[asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim)
hydrodynamics plugin and sets out some background theory to help compare
the approach with the standard linear potential flow model and other
implementations such as the [WEC-Sim Nonlinear Buoyancy and Froude-Krylov Excitation model](http://wec-sim.github.io/WEC-Sim/master/user/advanced_features.html#nonlinear-buoyancy-and-froude-krylov-excitation).

## Linear model

In the linear potential flow model the fluid pressure is given by the
linearised Bernoulli equation:

```math
\begin{equation}
p = - \rho \left(\frac{\partial \phi}{\partial t} + g z\right).
\end{equation}
```

The free surface boundary condition for a progressive plane wave is:

```math
\begin{equation}
\eta(x, t) = - \frac{1}{g}\left.\frac{\partial \phi}{\partial t} \right|_{z=0},
\end{equation}
```

which has the solution:

```math
\begin{equation}
\phi(x, z, t) = \textrm{Re}\left(A e^{i\omega t}\varphi (x, z)\right),
\end{equation}
```

with:

```math
\begin{equation}
\varphi(x, z) = \left(\frac{i g}{\omega}\right) e^{kz -ikx}.
\end{equation}
```

Notes:

- We use a RH coordinate system with $z$-up, $x$ to the right.
- For clarity we omit $y$ and give the 2d form of the equations.
- $g$ is acceleration due to gravity.
- $\rho$ is the fluid density.
- $\omega = 2\pi/T$ is the angular temporal frequency.
- $k = k(\omega)$ is the angular spatial wavenumber.
- $A$ is the wave amplitude. We may also use the wave height $H=2A$.
- The linearised free surface boundary condition for the wave applies at $z=0$.

### Forces

When analysing wave body interactions in the linear model it is usual to
decompose the total potential into contributions arising from the motions of
the body's various degrees of freedom and contributions from the wave field.
The latter is further subsdivided into two components comprising the incident
and scattered waves.

In the following discussion we focus on the contribution to the
potential from the undisturbed incident wave field $\phi_I(x, t)$ and 
will drop the subscript $I$.

The force acting on the body arising from the pressure field is:

```math
\begin{equation}
\boldsymbol{F} = \int_{S_B}\, p\, \boldsymbol{n}\, dS,
\end{equation}
```

where the surface integral is over the submerged surface $S_{B}$ and
$\boldsymbol{n}$ is normal to the surface facing out of the fluid
(and so into the floating body). Substituting the equation for pressure:

```math
\begin{equation}
\boldsymbol{F} = - \underbrace{\rho \int_{S_B} \frac{\partial \phi}{\partial t}\,
\boldsymbol{n}\, dS}_{\text{Froude-Krylov force}}
- \underbrace{\rho g \int_{S_B} z\, \boldsymbol{n}\, dS}_{\text{buoyancy force}},
\end{equation}
```

Here the force is decomposed into two parts: a hydrostatic buoyancy force
and hydrodynamic pressure force named the Froude-Krylov force.

The linear model then proceeds to evaluate these integrals by using Gauss's 
divergence theorem and assumptions about linearity and small displacements to replace the integral over the dynamic submerged surface $S_B$ with
integrals over the equilibrium displaced volume $V_o$ and
waterplane surface $S_o$.

Rather than follow this approach, we approximate the body with a
triangular mesh and evaluate the integrals directly by summing over the
contributions from each triangle. This captures some nonlinear effects, even
though the model for the incident wave potential remains linear.

## Nonlinear effects

By summing over the contribution from each submerged mesh element we can
relax two assumptions from the linear model:

- Small displacements - the body may be translated and rotated arbitrarily.
- Initial submerged surface - the instantaneous submerged surface is used when
  computing the pressure integrals.

A key assumption in what follows is that the solution for the
linear wave potential with the boundary condition for the free surface applied
at $z=0$ is also a valid solution when the boundary condition is applied on the
curvilinear surface $z = \eta(x, t)$.

Then we can do the following: let $z' = z - \eta(x, t)$ and suppose the
potential is valid for $z'$,

```math
\begin{equation}
\varphi(x, z') = \left(\frac{i g}{\omega}\right) e^{kz' -ikx},
\end{equation}
```

with the boundary condition now,

```math
\begin{equation}
\eta(x, t) = - \frac{1}{g}\left.\frac{\partial \phi}{\partial t} \right|_{z'=0},
\end{equation}
```

We can use this to write:

```math
\begin{equation}
- \rho\, \frac{\partial \phi}{\partial t}
= \rho g\, \textrm{Re}\left(A e^{i\omega t} e^{kz' -ikx}\right)
= \rho g\, \textrm{Re}\left(A e^{kz'} e^{-ikx + i\omega t}\right)
= \rho g\, e^{kz'}\eta(x, t),
\end{equation}
```

and so the pressure is then:

```math
\begin{align}
p  = - \rho \left(\frac{\partial \phi}{\partial t} + g z\right)
  &= \rho g\, e^{kz'}\eta(x, t) - \rho g z \nonumber \\
  &= \rho g\, (e^{kz'} - 1)\, \eta(x, t)
  - \rho g\, (z - \eta(x, t)) \nonumber \\
  &= \rho g\, (e^{kz'} - 1)\, \eta(x, t)
  - \rho g z',
\end{align}
```

The forces are:

```math
\begin{align}
\boldsymbol{F} &=
\rho g \int_{S_B} (e^{kz'} - 1)\, \eta(x, t)\, \boldsymbol{n}\, dS
- \rho g \int_{S_B} z'\, \boldsymbol{n}\, dS
\nonumber \\
&\simeq 
\underbrace{
\rho g \sum_{S_{B, i}} (e^{kz'_{i}} - 1)\,
\eta(x_{i}, t)\, \boldsymbol{n}_{i}\, S_{i}
}_{\text{nonlinear Froude-Krylov force}}
- \underbrace{
\rho g  \sum_{S_{B, i}} z'_{i} \, \boldsymbol{n}_{i}\, S_{i}
}_{\text{nonlinear buoyancy force}}
\end{align}
```

In the last line the integrals are replaced by sums over the mesh
triangle elements. The wave elevation $\eta(x_{i}, t)$ and
depth below the wave surface $z' _ {i}$ are evaluated at the triangle centroid.
$S_i$ and $\boldsymbol{n} _ {i}$ are the triangle area and inward normal.


## WEC-Sim Nonlinear Buoyancy and Froude-Krylov calculations

The functions for wave elevation, nonlinear buoyancy and Froude-Krylov
forces are located in the Matlab modules:

- `WEC-Sim/source/simulink/model/waveElevation.m`
- `WEC-Sim/source/simulink/model/nonlinearBuoyancy.m`
- `WEC-Sim/source/simulink/model/nonFKForce.m`

These modules calculate the forces and moments for both regular and irregular
waves. In the following we only review the force calculation for regular waves
as the descriptions for moments and irregular waves are similar.

### Simulink conventions

- Simulink evolves a state vector for the body center of mass in the
  world frame.
- The displacement vector input to the `waveElevation` function is the
  origin of the body frame (the simulation displacement is offset by the
  body frame center of mass vector `cg`).
- Mesh files must have their origin located at the center of mass.
  The functions for wave elevation, nonlinear buoyancy, etc. transform
  the mesh centroids by the current pose of the body origin then offset
  by the constant vector `cg`. The result is the current position of
  each centroid in the world frame.

### Wave elevation

The module `waveElevation.m` calculates the wave elevation $\eta(x, t)$ for the
centroid of each triangle in a mesh. For regular waves directed along the
positive $x$ axis the formula is:

```math
\begin{equation}
\eta(x, t) = A \cos (k x - \omega t).
\end{equation}
```

### Nonlinear Buoyancy

The module `nonlinearBuoyancy.m` calculates the force corresponding to the
hydrostatic pressure term in the linearised Bernoulli equation (Eq. 1).

```math
\begin{align}
\boldsymbol{F}_{B} &=
- \rho g \int_{S_B} z\, \boldsymbol{n}\;dS
\nonumber \\
&\simeq 
- \rho g  \sum_{S_{B, i}} z_{i} \, \boldsymbol{n}_{i}\, S_{i}.
\end{align}
```

Only contributions from mesh elements with centroid coordinate
$z_i < \eta(x_i, t)$ are included in the sum. $S_i$ and $\boldsymbol{n} _ {i}$
are the triangle area and inward normal for each submerged mesh element.

### Nonlinear Froude-Krylov

The module `nonFKForce.m` calculates the nonlinear Froude-Krylov force
corresponding to the dynamic pressure term in the linearised Bernoulli 
equation (Eq. 1). The pressure is evaluated on the elements of the
instantaneous submerged boundary with the depth calculated using the
instantaneous wave surface as a reference:

```math
\begin{align}
\boldsymbol{F}_{FK}
&= - \left.\rho \int_{S_B} \frac{\partial \phi}{\partial t}
\boldsymbol{n}\, dS\right|_{z - \eta(x, t) = 0}
\nonumber \\
&\simeq \underbrace{
\rho g  \sum_{S_{B, i}} e^{z_{i} - \eta(x_{i}, t)}\, \eta(x_{i}, t)
\, \boldsymbol{n}_i\;S_{i}
}_{\text{nonlinear Froude-Krylov force}}.
\end{align}
```

There is a second term to act as an offset to the contribution from the
standard linear model via the `Regular Wave Excitation` block which remains
active in the Simulink model when the nonlinear model is enabled.

```math
\begin{align}
\boldsymbol{F}_{FK}
&= - \left.\rho \int_{S_B} \frac{\partial \phi}{\partial t}
\boldsymbol{n}\, dS\right|_{\eta(x, t)=0}
\nonumber \\
&= \underbrace{
\rho g  \sum_{S_{0, i}} e^{z_{0, i}}\, \eta(x_{0, i}, t)
\, \boldsymbol{n}_{0, i}\, S_{i}
}_{\text{linear Froude-Krylov force}}.
\end{align}
```

## Summary

#### Similarities

1. The net nonlinear contribution to the buoyancy and Froude-Krylov force in
WEC-Sim from (Eq. 13) and (Eq. 14) is the same as (Eq. 11).

#### Differences

1. There is no equivalent to the linear offset term in for the Froude-Krylov
force (Eq. 15) in [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) because the linear forces are completely disabled when using the non-linear
model.

1. The allocation between the nonlinear buoyancy and Froude-Krylov forces is
different. In [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) a
change of variables is used to redefine the free surface to correspond to the
instantaneous wave elevation and the pressure is calculated with
reference to this.

1. [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) includes a
remeshing algorithm to divide any mesh elements that intersect the surface
into below surface and above surface parts. This means that a good estimate
of the instantaneous submerged boundary can be obtained using a mesh with fewer
triangles.

1. In the nonlinear buoyancy calculation [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) calculates the center of pressure for each triangle
and applies the force at that point rather than at the centroid. This prevents
spurious moments appearing for asymmetrical meshes at equilibrium.
The calculation becomes less important as the mesh resolution is increased.


---

## Appendix A. Gazebo plugin development

1. The hydrodynamics plugin in the main branch of [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) computes the nonlinear buoyancy force
in (Eq. 11). The Froude-Krylov terrm is not calculated. The plugin may be used
with any of the available wave models, including irregular waves with
randomised amplitudes and phases summed using FFT methods.

1. A demonstration plugin for the linear wave-body model is available on the [demo/linear-wave-body](https://github.com/srmainwaring/asv_wave_sim/tree/demo/linear-wave-body) branch.

2. There is an experimental version of the model that includes the nonlinear Froude-Krylov term for regular waves in the [srmainwaring/demo/lwb-nonlinear-fk](https://github.com/srmainwaring/asv_wave_sim/tree/srmainwaring/demo/lwb-nonlinear-fk) branch.
