import multiprocessing as mp
import numpy as np
import time as t

globVal = np.array([1, 2, 3])

def f1(array, q):
    print("1 run")
    newarray = np.append(array, [4])
    t.sleep(2)
    q.put(newarray)
    print("1 done")


def f2(q):
    print("2 run")
    print("val", q.get())
    print("2 done first")
    print("val", q.get())
    print("2 done sec")

def f3():
    print("indep run")
    t.sleep(2)
    print("indep done")


if __name__ == '__main__':
    print("Running")
    ctx = mp.get_context('spawn')
    queue = ctx.Queue()
    p1 = ctx.Process(target=f1, args=(globVal, queue))
    p2 = ctx.Process(target=f2, args=(queue,))
    p3 = ctx.Process(target=f3)
    print("def done")
    p1.start()
    print("1 started")
    p2.start()
    print("2 started")

    # To hold prog to finish funcrun.
    p1.join()
    print("1 joined")
    p2.join(5)
    print("2 done or terminated")
    #if p2.is_alive():
    #    p2.terminate()
    p3.start()
    t.sleep(5)
    print("p1alive? ", p1.is_alive())
    print("p2alive? ", p2.is_alive())
    print("p3alive? ", p3.is_alive())
