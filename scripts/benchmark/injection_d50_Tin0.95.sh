#!/bin/bash

# don't change:
name="injection_d50_Tin0.95"

#
# USER SET:
#
bindir=/home/szellmann/point-clouds/build
outdir=/home/szellmann/point-clouds-benchmarks/${name}
data="--mmpld /home/szellmann/Downloads/injection_d50_Tin0.95_Tout1.05_vfeed0.002.tiff"
orbit_count="--orbit 50"
orbit_center="--orbit-center 100 197.501 100"
orbit_up="--orbit-up 0 -1 0"
orbit_radius="--orbit-radius 400"
rbegin=0.1
rend=5.0
rinc=0.1

##
scriptdir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

echo "Create if not exists (outdir): "${outdir}
mkdir -p ${outdir}
cp ${scriptdir}/${name}.ini ${outdir}/viewer.ini
cd ${outdir}
echo "Running benchmark in: "$PWD
for r in $(seq ${rbegin} ${rinc} ${rend})
do
  ${bindir}/viewer-headless     \
            ${data}             \
            ${orbit_count}      \
            ${orbit_center}     \
            ${orbit_up}         \
            ${orbit_radius}     \
            --radius $r         \
            --benchmark         \
        2>&1 | tee ${outdir}/${name}.out
done

python3 ${scriptdir}/plot.py ${outdir}/benchmark*.txt -o ${outdir}/plot_${name}.pdf
