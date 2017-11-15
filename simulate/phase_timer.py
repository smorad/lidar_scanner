import time

class Timer:
    def __enter__(self):
        self.t = time.time()

    def __exit__(self, *args):
        print('Took {:.02f} seconds'.format(time.time() - self.t))
        
