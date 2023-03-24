#!/bin/bash

# don't change:
name="boiler"

#
# USER SET:
#
bindir=/home/szellmann/point-clouds/build
outdir=/home/szellmann/point-clouds-benchmarks/${name}
data="--points /home/szellmann/Downloads/points/points/boiler/uintah"
orbit_count="--orbit 50"
orbit_center="--orbit-center 0.175227 0.29995 0.301985"
orbit_up="--orbit-up 0 -1 0"
orbit_radius="--orbit-radius 0.2"
rbegin=0.00007
rend=0.004
rinc=0.0000786

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
