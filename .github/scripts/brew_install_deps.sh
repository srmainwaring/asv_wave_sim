#!/bin/bash

# See the Jenkins script in gazebo-tooling/release-tools
# https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash

# Run brew update to get latest versions of formulae
brew update

# Don't let brew auto-update any more for this session
# to ensure consistency
export HOMEBREW_NO_AUTO_UPDATE=1
export HOMEBREW_NO_INSTALL_UPGRADE=1

# Run brew config to print system information
# brew config

# Run brew doctor to check for problems with the system
# brew doctor || echo MARK_AS_UNSTABLE

# get the osrf tap for gazebo 
brew tap osrf/simulation

# packages to install 
PKGS=(
  gz-harmonic
  cgal
  fftw
)

# install packages
for PKG in "${PKGS[@]}"
do
  brew install ${PKG}
done

# The GitHub actions cache does restore links, so we force
# (re-)linking on all dependencies of packages we install. 

for PKG in "${PKGS[@]}"
do
  # link package
  brew link $PKG

  DEPS=$(brew deps ${PKG})
  declare -a DEPS_ARRAY
  DEPS_ARRAY=(${DEPS})

  for DEP_PKG in "${DEPS_ARRAY[@]}"
  do
    # link package dependencies
     brew link $DEP_PKG
  done
done


# keg only packages to install - only link dependencies
PKGS_KEG_ONLY=(
  dartsim@6.10.0
)

# install packages
for PKG in "${PKGS_KEG_ONLY[@]}"
do
  brew install ${PKG}
done

# The GitHub actions cache does restore links, so we force
# (re-)linking on all dependencies of packages we install. 

for PKG in "${PKGS_KEG_ONLY[@]}"
do
  # keg only do not link package

  DEPS=$(brew deps ${PKG})
  declare -a DEPS_ARRAY
  DEPS_ARRAY=(${DEPS})

  for DEP_PKG in "${DEPS_ARRAY[@]}"
  do
    # link package dependencies
     brew link $DEP_PKG
  done
done
