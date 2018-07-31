import time, threading, _thread
from time import sleep
def watchdog_timer(state):
    time.sleep(3)
    if not state['completed']:
        _thread.interrupt_main()

def run_example():
    while True:
        state = {'completed': False}
        watchdog = threading.Thread(target=watchdog_timer, args=(state,))
        watchdog.daemon = True
        watchdog.start()
        try:
            x = countdown()
            state['completed'] = True
            return x
        except KeyboardInterrupt:
            # this would be the place to log the timeout event
            pass
        else:
            run_example()

def countdown():
    for n in range(5):
        print(n)
        sleep(0.5)
    return "hi"

print(run_example())