set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
set style line 2 lc rgb '#dd181f' lt 1 lw 2 pt 1 ps 1.5

set view equal xyz

# Plot collisions as spheres
splot 'random/random.collisions' using 1:2:3 title 'Obstacles' with points pt 7 ps 1.5 lc rgb '#0060ad'

# Plot waypoints as lines
replot 'random/random.route' using 1:2:3 title 'Route' with lines lc rgb '#dd181f'

set terminal pdf size 10,11
set output "random/random.pdf"
replot