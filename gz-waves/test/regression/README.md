# Ocean material lifetime regression tests

These focused tests have their own opt-in switch and do not require the full
gnuplot/performance test dependencies. They need a working Ogre2 render engine
and display (Xvfb with Mesa is sufficient on Linux). With the usual WaveSim
dependencies installed:

```sh
cmake -S gz-waves -B build -DBUILD_TESTING=OFF -DGZ_WAVES_BUILD_RENDERING_TESTS=ON
cmake --build build --target ocean_lifetime_test -j2
LD_LIBRARY_PATH="$PWD/build/lib:${LD_LIBRARY_PATH:-}" \
DYLD_LIBRARY_PATH="$PWD/build/lib:${DYLD_LIBRARY_PATH:-}" \
ctest --test-dir build/test/regression -R '^ocean_lifetime_' --output-on-failure
```

The visual/geometry cases cover borrowed and private materials, default clones,
replacement of an owned clone, and reassignment of the same owned pointer.
They check scene registration, release of references after destruction, and
repeated destruction. The scene test
checks the lifetime needed by the dynamic-geometry path: after the primary
visual is removed, its material clone must remain registered until the custom
ocean renderable is destroyed by scene teardown. It also checks initial clone
material properties. All checks use explicit exit codes (including in Release
builds), not disabled `assert` statements.

For a negative control, `build/bin/ocean_lifetime_test scene borrowed` intentionally
reproduces the unsafe GUI teardown order. It is not a passing CTest case.

These focused tests do not replace closing a running Gazebo GUI or stopping a
camera-equipped server with both dynamic-geometry and dynamic-texture waves.
They do not test Windows, native Metal, or changes to a source material after
it has been cloned.
