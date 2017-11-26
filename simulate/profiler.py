import simulate
import cProfile

# Get runtime performance so slowdown can be pinpointed and removed
cProfile.run('simulate.main()')

