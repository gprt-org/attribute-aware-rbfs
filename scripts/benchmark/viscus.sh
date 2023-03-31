#!/bin/bash

# don't change:
name="viscus"

#
# USER SET:
#
bindir=/home/natevm/git/point-clouds/build
outdir=/home/natevm/git/point-clouds-benchmarks/${name}
data="--vtk /home/natevm/data/scivis2016/smoothinglength_0.30/run01/"
orbit_count="--orbit 50"
orbit_center="--orbit-center 8.58307e-05 -0.000281096 5"
orbit_up="--orbit-up 0 0 -1"
orbit_radius="--orbit-radius 15"
rbegin=0.01
rend=0.5
# note, 50 steps
rinc=0.0098

##
scriptdir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

echo "Create if not exists (outdir): "${outdir}
mkdir -p ${outdir}

cd ${outdir}
mkdir -p varying-radius
cp ${scriptdir}/${name}.ini ${outdir}/varying-radius/viewer.ini
cd varying-radius
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
        2>&1 | tee ${outdir}/varying-radius/${name}.out
done

cd ${outdir}
mkdir -p varying-ppl
cp ${scriptdir}/${name}.ini ${outdir}/varying-ppl/viewer.ini
cd varying-ppl
echo "Running varying-ppl benchmark in: "$PWD
for ppl in 1 2 4 8 16
do
  ${bindir}/viewer-headless             \
            ${data}                     \
            ${orbit_count}              \
            ${orbit_center}             \
            ${orbit_up}                 \
            ${orbit_radius}             \
            --particles-per-leaf $ppl   \
            --benchmark                 \
        2>&1 | tee ${outdir}/varying-ppl/${name}.out
done

python3 ${scriptdir}/plot.py ${outdir}/varying-radius/benchmark*.txt -o ${outdir}/varying-radius/plot_${name}.pdf
python3 ${scriptdir}/plot.py ${outdir}/varying-ppl/benchmark*.txt -o ${outdir}/varying-ppl/plot_${name}.pdf