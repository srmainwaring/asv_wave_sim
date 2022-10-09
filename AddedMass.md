# Gazebo / DART inertia conversion

An explanation of the Gazebo to DART inertial conversion and the different
conventions used by each system. The conclusion is that there is a missing
term when populating the DART spatial tensor with added mass because the
moments of inertia are taken about different axis.

## Drake labelling conventions

- https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__pose.html
- https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__inertia.html 

### Frame labels

- `W` - world frame.
- `B` - body frame.
- `Bcm` - body CoM frame.

### Point labels

- `Bo`  - body origin.
- `Bcm` - body center of mass.

### Pose

- `X_WB` - the pose of the body frame `B` in the world frame `W`.
- `X_BBcm` - the pose of the body CoM frame `Bcm` in the body frame `B`.


### Inertia matrix

The inertia matrix, taken about a point `P`, expressed in frame `F`.

- `I_BBcm_Bcm` - inertia matrix of body `B` about the body center of mass `Bcm` in frame `Bcm` 
- `I_BBcm_B` - inertia matrix of body `B` about the body center of mass `Bcm` in frame `B` 
- `I_BBo_B` - inertia matrix of body `B` about the body origin `Bo` in frame `B` 

### Position vectors

- `c_BoBcm_B` position vector from the body origin `Bo` to the body center of mass `Bcm` in the body frame `B`. 


## Gazebo inertial object

The SDF documentation http://sdformat.org/spec?ver=1.9&elem=link#inertial_inertia for the `<inertia>` element states:


> Description: This link's moments of inertia ixx, iyy, izz and products of inertia ixy, ixz, iyz about Co
> (the link's center of mass) for the unit vectors Ĉx, Ĉy, Ĉᴢ fixed in the center-of-mass-frame C.

This means that the object `gz::math::MassMatrix3` contains:

- `m` - the scalar mass.
- `I_BBcm_Bcm` - inertia matrix of body `B` about the body center of mass `Bcm` in frame `Bcm`.

This is used to construct the object `gz::math::Inertial` which is the inertial taken about point `Bcm`
for body `B` in frame `F`. Since the pose from SDFFormat is `X_BBcm`, the object stores internally:

- `m`
- `I_BBcm_Bcm`
- `X_BBcm`

The functions return the following:

- `MassMatrix -> (m, I_BBcm_Bcm)`
- `Pose -> (X_BBcm)`
- `Moi -> (I_BBcm_B = X_BBcm.R * I_BBcm_Bcm * X_BBcm.R^T)`

The key point to note is that when accessing the the moment of inertia it is transformed
to the body frame `B` but is taken about a point located at the CoM `P = Bcm`.

## DART spatial tensor

The Gazebo function we are interested in is: `gz::physics::dartsim::SDFFeatures::ConstructSdfLink`
where `gz::math::Inertial` is translated into `dart::dynamics::Inertia`.

### 1. First examine the case when there is no fluid added mass:

```c++
  const gz::math::Inertiald &sdfInertia = _sdfLink.Inertial();
  bodyProperties.mInertia.setMass(sdfInertia.MassMatrix().Mass());

  const Eigen::Matrix3d I_link = math::eigen3::convert(sdfInertia.Moi());

  bodyProperties.mInertia.setMoment(I_link);

  const Eigen::Vector3d localCom =
      math::eigen3::convert(sdfInertia.Pose().Pos());

  bodyProperties.mInertia.setLocalCOM(localCom);
```

Break down what is going on:

- Set `m` - the scalar mass `m`.
- Set `I_BBcm_B` - the moment of inertia of the body `B`, taken about the body CoM `Bcm` in frame `B`.
- Set `c_BoBcm_B` - position vector from the body origin `Bo` to the body center of mass `Bcm` in the body frame `B`.

From these inputs DART, calculates the spatial tensor for rigid body rotations about the **body origin** `Bo`.

This is the spatial inertial tensor from Roy Featherstone, Rigid Body Dynamics Algorithms, §2.13, p33, Eq(2.63) Springer, 2008. Internally DART computes the spatial tensor to have the following elements:

- `TL  = I_BBo_B = I_BBcm_B + m * cx * cx^T`
- `TR  = m * cx`
- `BL  = m * cx^T`
- `BR =  m * Identity(3)`

where `cx` is the skew-symmetric operator created from `c_BoBcm_B`.

Compare this to the definition for the Gazebo body matrix returned by: `gz::math::Inertial::BodyMatrix`:

- `BR  = I_BBcm_B
- `BL  = m * cx`
- `TR  = m * cx^T`
- `TL =  m * Identity(3)`

which is for rigid body rotations about the **body center of mass** `Bm`.

Now in this case, the different convention for the axis of rotation does not matter because Gazebo does not use
the `BodyMatrix` function to set the DART spatial inertial tensor, and the change of axis is calculated internally
by DART. However this is not the case when added mass is considered.

### 2. With fluid added mass 

In this case the DART spatial inertial tensor is first calculated as in case 1., then a spatial inertial tensor
for the fluid added mass is calculated and the sum of the two is set using
`dart::dynamics::Inertia::setSpatialTensor`:

```c++
    bodyProperties.mInertia.setSpatialTensor(
        bodyProperties.mInertia.getSpatialTensor() +
        math::eigen3::convert(featherstoneMatrix)
```

So the important note here is that the featherstoneMatrix must be for rotations
about the body orgin `Bo` and with respect to the body frame `B`.


While not clearly documented in the Gazebo class, it is suggested by the function:

```c++
      /// \brief Spatial mass matrix, which includes the body's inertia, as well
      /// as the inertia of the fluid that is dislocated when the body moves.
      /// The matrix is expressed in the object's frame F, not to be confused
      /// with the center of mass frame Bi.
      /// \return The spatial mass matrix.
      /// \sa BodyMatrix
      /// \sa FluidAddedMass
      public: Matrix6<T> SpatialMatrix() const
      {
        return this->addedMass.has_value() ?
            this->BodyMatrix() + this->addedMass.value() : this->BodyMatrix();
      }
```

that the Gazebo fluid added mass is expected to be computed in the body frame `B` for rotations about
the body center of mass `Bcm` (because the quantities are being summed, and this is consistent with
the other interfaces to this class that return moment of intertia etc.)

## Conclusion

The above suggests that there is a term missing in the current calculation of the featherstone matrix
in `gz::physics::dartsim::SDFFeatures::ConstructSdfLink` to account for the change of axis
from `Bcm` to `Bo`.

For fluid added mass the mass matrix is not diagonal with equal entries, so we can't calculate
`m * cx * cx^T` as for the inertial term. My suggestion is to use:

```
I_BBo_B = I_BBcm_B + cx * M * cx^T
```

and deduce `CMCT = cx * M * cx^T`  using:

```c++
    auto M = sdfInertia.FluidAddedMass().value().Submatrix(
        math::Matrix6d::TOP_LEFT);

    auto CM = sdfInertia.FluidAddedMass().value().Submatrix(
        math::Matrix6d::BOTTOM_LEFT);

    auto invM = M.Inverse();
    auto C = CM * invM;
    auto CMCT = CM * C.Transposed();
```