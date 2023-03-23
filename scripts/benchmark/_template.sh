#!/bin/bash

# don't change:
name="_template"

#
# USER SET:
#
bindir=/home/szellmann/point-clouds/build
outdir=/home/szellmann/point-clouds-benchmarks/${name}
orbit_count="--orbit 50"
orbit_center="--orbit-center 0.000796274 6.25849e-07 0"
orbit_up="--orbit-up 0 1 0"
orbit_radius="--orbit-radius 10"

##
scriptdir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

echo "Create if not exists (outdir): "${outdir}
mkdir -p ${outdir}
cp ${scriptdir}/${name}.ini ${outdir}/viewer.ini
cd ${outdir}
echo "Running benchmark in: "$PWD
${bindir}/viewer-headless ${orbit_count} ${orbit_center} ${orbit_up} ${orbit_radius} --benchmark
