import thread
import time

def input_thread(L):
    print "sup"
    raw_input()
    L.append(1)

def do_print():
    L = []

    thread.start_new_thread(input_thread, (L,))
    while 1:
        time.sleep(.1)
        if L: break
        print "Hi"

do_print()

